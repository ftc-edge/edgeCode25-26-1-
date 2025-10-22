package org.firstinspires.ftc.teamcode.yaiba; // <--- adjust to your package

import android.util.Log;

import org.tensorflow.lite.Interpreter;

import java.util.ArrayList;
import java.util.List;

/**
 * ModelInputMapper
 *
 * Utility to map a single flat observation vector (float[]) into the exact input
 * arrays expected by a TensorFlow Lite Interpreter (supports up to 4D inputs common in RL models).
 *
 * Usage:
 *   ModelInputMapper.MapperResult res =
 *       ModelInputMapper.createInputsFromFlat(interpreter, flatObs);
 *   if (!res.warnings.isEmpty()) { /* log or telemetry them }*/
//        *   Object[] inputs = res.inputs; // feed this into interpreter.runForMultipleInputsOutputs(inputs, outputsMap);
// *
//         * Notes:
//        *  - This helper assumes FLOAT32 inputs.
// *  - If your model uses quantized inputs (INT8/UINT8) you must quantize manually (not handled here).
//        *  - If your model has signature-based inputs instead of positional tensors, you must use signature runner instead.
// */
public class ModelInputMapper {
    private static final String TAG = "ModelInputMapper";

    /** Result container returned by createInputsFromFlat(...) */
    public static class MapperResult {
        public final Object[] inputs;   // ready-to-pass Object[] for interpreter.runForMultipleInputsOutputs
        public final List<String> warnings; // non-empty if we trimmed/padded or something suspicious
        public final int expectedTotal; // total number of floats expected across all inputs (excl batch dim)
        public final int providedTotal; // flatObs length

        MapperResult(Object[] inputs, List<String> warnings, int expectedTotal, int providedTotal) {
            this.inputs = inputs;
            this.warnings = warnings;
            this.expectedTotal = expectedTotal;
            this.providedTotal = providedTotal;
        }
    }

    /**
     * Main helper: create input arrays matching the interpreter's input tensor shapes and
     * populate them from a single flat observation vector (flatObs).
     *
     * Behavior:
     *  - If flatObs length > required total: the extra values are TRIMMED (end trimmed).
     *  - If flatObs length < required total: remaining entries are ZERO-PADDED.
     *
     * Supported input tensor ranks: 1..4 (where rank=1 means shape like [N] and rank=2 typical [1,N]).
     *
     * @param interpreter TensorFlow Lite Interpreter already created and ready.
     * @param flatObs     flat float array containing concatenated features (order must match model training).
     * @return MapperResult containing Object[] inputs and warnings.
     */
    public static MapperResult createInputsFromFlat(Interpreter interpreter, float[] flatObs) {
        if (interpreter == null) {
            throw new IllegalArgumentException("Interpreter is null");
        }
        if (flatObs == null) flatObs = new float[0];

        int nInputs = interpreter.getInputTensorCount();
        Object[] inputs = new Object[nInputs];
        List<String> warnings = new ArrayList<>();

        // Calculate sizes and allocate
        int totalExpected = 0;
        int[] perInputSizes = new int[nInputs];
        int[][] shapes = new int[nInputs][]; // full shape for each input (including batch dim if present)

        for (int i = 0; i < nInputs; i++) {
            org.tensorflow.lite.Tensor t = interpreter.getInputTensor(i);
            int[] shape = t.shape(); // may throw on some builds; assume available if you used inspector earlier.
            shapes[i] = shape;
            // compute number of elements excluding a batch dimension if present
            int startDim = (shape.length >= 2 && shape[0] == 1) ? 1 : 0;
            int size = 1;
            for (int d = startDim; d < shape.length; d++) size *= shape[d];
            perInputSizes[i] = size;
            totalExpected += size;
        }

        // Now split flatObs into perInputSizes, trimming or padding as necessary
        int provided = flatObs.length;
        if (provided < totalExpected) {
            warnings.add(String.format("Provided obs length %d < expected %d. Padding with zeros.", provided, totalExpected));
        } else if (provided > totalExpected) {
            warnings.add(String.format("Provided obs length %d > expected %d. Trimming extra values.", provided, totalExpected));
        }

        int cursor = 0;
        for (int i = 0; i < nInputs; i++) {
            int[] shape = shapes[i];
            int size = perInputSizes[i];
            // Create appropriate array by rank (support ranks 1..4)
            int rank = shape.length;
            boolean hasBatch = (rank >= 2 && shape[0] == 1);
            try {
                if (rank == 1) {
                    // shape like [N] -> float[N]
                    float[] arr = new float[shape[0]];
                    fillFromFlat(arr, 0, flatObs, cursor, size);
                    cursor += Math.min(size, provided - cursor);
                    inputs[i] = arr;
                } else if (rank == 2) {
                    // shape like [1, N] or [M,N] -> float[shape[0]][shape[1]]
                    int dim0 = shape[0];
                    int dim1 = shape[1];
                    float[][] arr2 = new float[dim0][dim1];
                    // fill row-major
                    for (int a = 0; a < dim0; a++) {
                        for (int b = 0; b < dim1; b++) {
                            arr2[a][b] = getFlatValueOrZero(flatObs, cursor, provided, totalExpected);
                            cursor++;
                        }
                    }
                    inputs[i] = arr2;
                } else if (rank == 3) {
                    // shape [1, H, W] or [H,W,C] -> float[dim0][dim1][dim2]
                    int d0 = shape[0];
                    int d1 = shape[1];
                    int d2 = shape[2];
                    float[][][] arr3 = new float[d0][d1][d2];
                    for (int a = 0; a < d0; a++) {
                        for (int b = 0; b < d1; b++) {
                            for (int c = 0; c < d2; c++) {
                                arr3[a][b][c] = getFlatValueOrZero(flatObs, cursor, provided, totalExpected);
                                cursor++;
                            }
                        }
                    }
                    inputs[i] = arr3;
                } else if (rank == 4) {
                    // shape [1, H, W, C] typical image -> float[dim0][dim1][dim2][dim3]
                    int d0 = shape[0];
                    int d1 = shape[1];
                    int d2 = shape[2];
                    int d3 = shape[3];
                    float[][][][] arr4 = new float[d0][d1][d2][d3];
                    for (int a = 0; a < d0; a++) {
                        for (int b = 0; b < d1; b++) {
                            for (int c = 0; c < d2; c++) {
                                for (int d = 0; d < d3; d++) {
                                    arr4[a][b][c][d] = getFlatValueOrZero(flatObs, cursor, provided, totalExpected);
                                    cursor++;
                                }
                            }
                        }
                    }
                    inputs[i] = arr4;
                } else {
                    // Unhandled rank > 4: fallback to a 1D float array of required length
                    float[] arr = new float[size];
                    fillFromFlat(arr, 0, flatObs, cursor, size);
                    cursor += Math.min(size, provided - cursor);
                    inputs[i] = arr;
                    warnings.add("Input tensor rank " + rank + " not fully supported; flattened to 1D.");
                }
            } catch (Exception e) {
                // Defensive: if anything fails, create a flat float[size] and continue
                Log.w(TAG, "Failed to create shaped array for input " + i + " (shape=" + java.util.Arrays.toString(shape) + "): " + e.toString());
                float[] arr = new float[size];
                fillFromFlat(arr, 0, flatObs, cursor, size);
                cursor += Math.min(size, provided - cursor);
                inputs[i] = arr;
                warnings.add("Fallback: used flattened array for input " + i + " due to exception: " + e.getMessage());
            }
        }

        // If cursor < totalExpected, we've padded with zeros; if cursor > totalExpected that's impossible here
        return new MapperResult(inputs, warnings, totalExpected, provided);
    }

    // Helper: get value or 0 if flatObs exhausted. We cap reads to avoid IndexOutOfBounds.
    private static float getFlatValueOrZero(float[] flatObs, int cursor, int provided, int totalExpected) {
        if (flatObs == null) return 0f;
        if (cursor < provided) return flatObs[cursor];
        return 0f;
    }

    // Helper: fill a 1D array segment from flatObs starting at flatCursor. Does NOT exceed flatObs length.
    private static void fillFromFlat(float[] target, int targetOffset, float[] flatObs, int flatCursor, int count) {
        int provided = (flatObs == null) ? 0 : flatObs.length;
        for (int i = 0; i < count; i++) {
            int flatIndex = flatCursor + i;
            float v = (flatIndex < provided) ? flatObs[flatIndex] : 0f;
            target[targetOffset + i] = v;
        }
    }
}
