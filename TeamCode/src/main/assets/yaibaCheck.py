import onnxruntime as ort
import numpy as np

session = ort.InferenceSession("BODY.onnx")

# Test 1: Same input multiple times (tests if stochastic)
obs = np.array([[0.5, 0.5, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

print("=== Test 1: Determinism (same input) ===")
for i in range(5):
    result = session.run(None, {"obs_0": obs})
    print(f"Run {i+1}: {result}")
    print(f"  Shape: {result[0].shape}")
    print(f"  Values: {result[0]}")

# Test 2: Different inputs (tests if model responds to changes)
print("\n=== Test 2: Responsiveness (different inputs) ===")

obs1 = np.array([[1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
obs2 = np.array([[0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32)
obs3 = np.array([[-1.0, 0.0, 0.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0]], dtype=np.float32)

result1 = session.run(None, {"obs_0": obs1})
result2 = session.run(None, {"obs_0": obs2})
result3 = session.run(None, {"obs_0": obs3})

print(f"Input [1.0, 0.0, ...]: {result1[0]}")
print(f"Input [0.0, 1.0, ...]: {result2[0]}")
print(f"Input [-1.0, 0.0, ...]: {result3[0]}")

# Test 3: Check strafe output specifically
print("\n=== Test 3: Strafe Output Analysis ===")
print("First, let's understand the output shape:")
test_result = session.run(None, {"obs_0": obs})
print(f"Output type: {type(test_result)}")
print(f"Output length: {len(test_result)}")
print(f"First element type: {type(test_result[0])}")
print(f"First element shape: {test_result[0].shape}")
print(f"First element: {test_result[0]}")

# Now adapt based on actual shape
print("\nTesting 10 random inputs:")
for i in range(10):
    obs = np.random.randn(1, 9).astype(np.float32)
    result = session.run(None, {"obs_0": obs})
    
    # Flexible indexing based on shape
    output = result[0]
    if output.ndim == 2:  # Shape is [1, 3]
        strafe, forward, rot = output[0][0], output[0][1], output[0][2]
    elif output.ndim == 1:  # Shape is [3]
        strafe, forward, rot = output[0], output[1], output[2]
    else:
        print(f"Unexpected shape: {output.shape}")
        continue
    
    print(f"Obs: {obs[0][:3]} → strafe={strafe:.6f}, forward={forward:.6f}, rot={rot:.6f}")