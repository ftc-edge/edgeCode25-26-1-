# inspect_tflite.py
# Usage:
#   python inspect_tflite.py /path/to/BODY.tflite
#
# This script:
# - prints input/output tensor details
# - searches the flatbuffer for "Flex" op names
# - tries a test inference (with zeros) and prints outputs (if possible)
#
# Notes:
# - Requires TensorFlow (pip install tensorflow) OR tflite-runtime.
# - If interpreter.invoke() fails due to Flex, you'll see the exception message.

import sys
import os
import struct

def find_bytes(filename, keyword_bytes, context=32, max_hits=20):
    hits = []
    with open(filename, "rb") as f:
        data = f.read()
    start = 0
    while True and len(hits) < max_hits:
        idx = data.find(keyword_bytes, start)
        if idx == -1:
            break
        before = max(0, idx - context)
        after = min(len(data), idx + len(keyword_bytes) + context)
        snippet = data[before:after]
        hits.append((idx, snippet))
        start = idx + 1
    return hits

def print_hex_snippet(snippet):
    # try to print ascii-friendly hex + printable characters
    try:
        readable = ''.join((chr(b) if 32 <= b < 127 else '.') for b in snippet)
    except Exception:
        readable = repr(snippet)
    hexpart = ' '.join(f"{b:02x}" for b in snippet[:64])
    print("  hex:", hexpart)
    print("  ascii:", readable)

def try_tflite_invoke(model_path):
    # prefer tensorflow.lite.Interpreter, fall back to tflite_runtime if available
    try:
        import tensorflow as tf
        Interpreter = tf.lite.Interpreter
    except Exception as e_tf:
        try:
            # try tflite_runtime
            from tflite_runtime.interpreter import Interpreter
        except Exception as e_rt:
            print("Neither tensorflow nor tflite_runtime available (or import failed).")
            print("Install tensorflow (pip install tensorflow) or tflite-runtime and retry.")
            raise

    print("\n[INFO] Creating TFLite Interpreter (desktop)...")
    try:
        interpreter = Interpreter(model_path=model_path)
    except Exception as e:
        print("[ERROR] Failed to create Interpreter:", e)
        raise

    try:
        interpreter.allocate_tensors()
    except Exception as e:
        print("[ERROR] allocate_tensors() failed:", e)
        # maybe still try to get input/output details
    try:
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()
    except Exception as e:
        print("[ERROR] Couldn't retrieve input/output details:", e)
        input_details = []
        output_details = []

    print("\nInputs:")
    for i, inp in enumerate(input_details):
        name = inp.get('name', '<noname>')
        shape = inp.get('shape', '<unknown>')
        dtype = inp.get('dtype', '<unknown>')
        print(f"  [{i}] index={inp.get('index')} name={name} shape={shape} dtype={dtype}")

    print("\nOutputs:")
    for i, out in enumerate(output_details):
        name = out.get('name', '<noname>')
        shape = out.get('shape', '<unknown>')
        dtype = out.get('dtype', '<unknown>')
        print(f"  [{i}] index={out.get('index')} name={name} shape={shape} dtype={dtype}")

    if len(input_details) == 0:
        print("\n[WARN] No input details found, skipping invocation test.")
        return interpreter, input_details, output_details

    # Build zero inputs according to shapes
    import numpy as np
    inputs = []
    for inp in input_details:
        s = inp.get('shape')
        if s is None:
            s = [1,]
        # ensure batch dimension exists; if shape has 0 in dims, replace with 1
        s = [ (1 if (isinstance(d, int) and d <= 0) else d) for d in s ]
        try:
            arr = np.zeros(tuple(s), dtype=np.float32)
        except Exception:
            # fallback to 1D
            arr = np.zeros((1,), dtype=np.float32)
        inputs.append((inp, arr))

    print("\n[INFO] Attempting to run a test inference with zero inputs.")
    for inp, arr in inputs:
        try:
            interpreter.set_tensor(inp['index'], arr)
        except Exception as e:
            print(f"  [WARN] set_tensor failed for input index {inp['index']}: {e}")

    try:
        interpreter.invoke()
        print("  [OK] interpreter.invoke() succeeded.")
        outputs_values = []
        for out in output_details:
            try:
                val = interpreter.get_tensor(out['index'])
                outputs_values.append(val)
            except Exception as e:
                outputs_values.append(f"<get_tensor error: {e}>")
        print("\nTest inference outputs (desktop):")
        for i, val in enumerate(outputs_values):
            print(f"  output[{i}]: shape={getattr(val,'shape',None)} type={type(val)}")
            # print small arrays fully, larger arrays with summary
            try:
                import numpy as np
                if isinstance(val, np.ndarray):
                    flat = val.flatten()
                    print("   values (first 10):", flat[:10])
                else:
                    print("   value:", val)
            except Exception:
                print("   (couldn't pretty-print value)")
    except Exception as e:
        print("  [ERROR] interpreter.invoke() failed:", e)
        print("  If the error mentions 'Flex' or unknown ops, the model requires the Flex runtime.")
    return interpreter, input_details, output_details

def main():
    if len(sys.argv) < 2:
        print("Usage: python inspect_tflite.py /path/to/BODY.tflite")
        sys.exit(1)

    model_path = sys.argv[1]
    if not os.path.isfile(model_path):
        print("File not found:", model_path)
        sys.exit(1)

    print("Inspecting:", model_path)
    # quick byte search for Flex-related strings
    keywords = [b"Flex", b"RandomStandardNormal", b"RandomStandardNormalV2", b"FlexRandomStandardNormal", b"random_normal"]
    found_any = False
    for kw in keywords:
        hits = find_bytes(model_path, kw, context=48, max_hits=10)
        if hits:
            found_any = True
            print(f"\n[FOUND] bytes match for '{kw.decode(errors='ignore')}' ({len(hits)} hits). Showing context snippets:")
            for i, (idx, snippet) in enumerate(hits):
                print(f"  hit {i+1}: offset {idx}")
                print_hex_snippet(snippet)
    if not found_any:
        print("\n[INFO] No raw 'Flex' strings found by simple byte scan. (Note: this is not 100% guaranteed.)")

    # Try tflite interpreter invoke on desktop
    try:
        try_tflite_invoke(model_path)
    except Exception as e:
        print("\n[INFO] Interpreter test aborted due to error:", e)
        print("[TIP] If interpreter.invoke() fails on desktop with Flex errors, you must re-export the model without Flex ops.")

if __name__ == "__main__":
    main()
