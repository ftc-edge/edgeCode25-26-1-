import sys
from pathlib import Path

try:
    import onnxruntime as ort
except Exception as exc:
    raise SystemExit(
        "onnxruntime is required. Install with: pip install onnxruntime"
    ) from exc


def main(model_path: Path) -> None:
    if not model_path.exists():
        raise SystemExit(f"Model not found: {model_path}")

    sess = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])

    print("Outputs (index -> name, shape, type):")
    for idx, out in enumerate(sess.get_outputs()):
        shape = out.shape
        dtype = out.type
        print(f"{idx}: {out.name}  shape={shape}  type={dtype}")

    print("\nInputs (name, shape, type):")
    for inp in sess.get_inputs():
        print(f"{inp.name}  shape={inp.shape}  type={inp.type}")


if __name__ == "__main__":
    if len(sys.argv) > 1:
        model = Path(sys.argv[1])
    else:
        model = Path(__file__).resolve().parent / "BODY.onnx"
    main(model)
