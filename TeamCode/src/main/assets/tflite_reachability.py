# tflite_reachability.py
# Usage:
#   python tflite_reachability.py /path/to/BODY.tflite

import sys
import os

def abort(msg):
    print("ERROR:", msg)
    sys.exit(1)

try:
    # TensorFlow pip wheel includes the generated schema module
    from tensorflow.lite.python import schema_py_generated as schema
except Exception as e:
    abort("Could not import TensorFlow TFLite schema. Make sure tensorflow is installed in this Python environment.\nImport error: " + str(e))

import flatbuffers

def load_model(buf):
    # Correct API: schema.Model.GetRootAsModel
    return schema.Model.GetRootAsModel(buf, 0)

def main():
    if len(sys.argv) < 2:
        print("Usage: python tflite_reachability.py /path/to/BODY.tflite")
        sys.exit(1)

    path = sys.argv[1]
    if not os.path.isfile(path):
        abort(f"File not found: {path}")

    with open(path, "rb") as f:
        data = f.read()

    model = load_model(data)
    # assume single subgraph model
    subgraph = model.Subgraphs(0)
    inputs = [subgraph.Inputs(i) for i in range(subgraph.InputsLength())]
    outputs = [subgraph.Outputs(i) for i in range(subgraph.OutputsLength())]

    # operator codes (maps codeIndex -> op description (builtin or custom))
    opcodes = []
    for i in range(model.OperatorCodesLength()):
        oc = model.OperatorCodes(i)
        custom_bytes = oc.CustomCode()
        custom = custom_bytes.decode('utf-8') if custom_bytes else None
        builtin = oc.BuiltinCode()
        opcodes.append((builtin, custom))

    # build operator list: index -> (opcode_index, inputs[], outputs[])
    operators = []
    for i in range(subgraph.OperatorsLength()):
        op = subgraph.Operators(i)
        code_index = op.OpcodeIndex()
        ins = [op.Inputs(j) for j in range(op.InputsLength())]
        outs = [op.Outputs(j) for j in range(op.OutputsLength())]
        operators.append((code_index, ins, outs))

    # Find operator indices whose custom code contains 'Flex' or 'Random'
    suspected_ops = []
    for op_idx, (code_index, ins, outs) in enumerate(operators):
        builtin, custom = opcodes[code_index]
        if custom and ("flex" in custom.lower() or "random" in custom.lower()):
            suspected_ops.append((op_idx, code_index, custom))

    print("Model path:", path)
    print("Subgraph inputs:", inputs)
    print("Subgraph outputs:", outputs)
    print("Operator code count:", len(opcodes))
    print("Operator count:", len(operators))
    print()

    if not suspected_ops:
        print("No operator with 'Flex'/'Random' pattern found in operator codes.")
    else:
        print("Suspected Flex/Random operators (op_index, opcode_index, customName):")
        for item in suspected_ops:
            print(" ", item)
    print()

    # Build maps
    tensor_producer = {}
    tensor_consumers = {}
    for op_idx, (code_index, ins, outs) in enumerate(operators):
        for t in outs:
            tensor_producer[t] = op_idx
        for t in ins:
            tensor_consumers.setdefault(t, set()).add(op_idx)

    # Backward traversal from outputs
    from collections import deque
    ops_reachable = set()
    tensors_reachable = set(outputs[:])
    q = deque(outputs[:])
    while q:
        tensor_idx = q.popleft()
        prod = tensor_producer.get(tensor_idx, None)
        if prod is not None:
            if prod not in ops_reachable:
                ops_reachable.add(prod)
                _, prod_ins, _ = operators[prod]
                for tin in prod_ins:
                    if tin not in tensors_reachable:
                        tensors_reachable.add(tin)
                        q.append(tin)

    print("Reachable operator count:", len(ops_reachable))
    print("Reachable tensor count:", len(tensors_reachable))
    print()

    # Check intersection with suspected ops
    flex_reachable = []
    for (op_idx, code_index, custom) in suspected_ops:
        if op_idx in ops_reachable:
            flex_reachable.append((op_idx, code_index, custom))
    if flex_reachable:
        print("***** FLEX OPERATORS ARE REACHABLE FROM MODEL OUTPUTS *****")
        print("These Flex/Random operators are on a path to outputs (thus affect inference):")
        for it in flex_reachable:
            print(" ", it)
        print("\nThis means the model's deterministic output depends on Flex op(s).")
        print("You should re-export the model without Flex ops (or run inference off-device).")
    else:
        print("No Flex/Random operator that we detected is reachable from outputs.")
        print("This means the Flex op(s) are likely dead/unreachable for the exported inference subgraph.")
        print("In that case we have safer options to try (see notes below).")

    # Diagnostics: show a few reachable ops
    print("\nSample reachable ops (op_idx -> opcode(custom)): ")
    sample = list(ops_reachable)[:30]
    for op_idx in sample:
        code_index, ins, outs = operators[op_idx]
        builtin, custom = opcodes[code_index]
        print(f" op {op_idx}: opcode_index={code_index} custom={custom} builtin={builtin} inputs={ins} outputs={outs}")

    # Producers for outputs
    print("\nProducers for output tensors:")
    for out in outputs:
        prod = tensor_producer.get(out, None)
        print(f" output_tensor {out} produced by op {prod}")

if __name__ == "__main__":
    main()
