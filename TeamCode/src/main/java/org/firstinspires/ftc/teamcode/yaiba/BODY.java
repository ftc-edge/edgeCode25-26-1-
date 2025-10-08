package org.firstinspires.ftc.teamcode.yaiba;

import android.content.Context;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;
import org.tensorflow.lite.flex.FlexDelegate;

import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.util.HashMap;
import java.util.Map;

public class BODY {

    private final Interpreter tflite;

    // --- MEAN arrays (Sub constants) ---
    private static final float[] OBS1_MEAN = {
            1.8187768f, 79.99787f, 8.2030687f, 59.998695f,
            0.99997884f, -6.3842688f, 19.999468f, 0f
    };
    private static final float[] OBS0_MEAN = {
            0f, 0.5204003f, 0.0004512872f, 0.51680624f, 0.0001276195f,
            0.52170676f, 0f, 0.51885885f, 0f, 0.51885885f
    };

    // --- STD arrays (Div constants) ---
    private static final float[] OBS1_STD = {
            161.80991f, 0.027785482f, 177.19409f, 0.020811629f,
            0.00049046875f, 178.02434f, 0.0069544823f, 0.00034681606f
    };
    private static final float[] OBS0_STD = {
            0.00034681606f, 0.11696029f, 0.0212312f, 0.11983932f,
            0.01129752f, 0.11985586f, 0.00034681606f, 0.11696027f,
            0.00034681606f, 0.11696027f
    };

    private static final float CLIP_MIN = -5.0f;
    private static final float CLIP_MAX = 5.0f;

    public BODY(Context context) throws IOException {
        MappedByteBuffer modelBuffer;
        try {
            modelBuffer = FileUtil.loadMappedFile(context, "BODY.tflite");
        } catch (IOException e) {
            throw new IOException("Model not found in assets: " + e.getMessage(), e);
        }

        Interpreter.Options options = new Interpreter.Options();
        FlexDelegate flexDelegate = new FlexDelegate();
        options.addDelegate(flexDelegate);
        options.setNumThreads(1);   // keep single thread on Control Hub
        options.setUseNNAPI(false);

        try {
            tflite = new Interpreter(modelBuffer, options);
        } catch (RuntimeException e) {
            // surface details to caller
            throw new IOException("Failed to create TFLite Interpreter: " + e.getMessage(), e);
        }
//        try {
//            MappedByteBuffer modelBuffer = FileUtil.loadMappedFile(context, "BODY.tflite");
//            Interpreter.Options options = new Interpreter.Options();
//            options.setNumThreads(1);
//            options.setUseNNAPI(false);
//            tflite = new Interpreter(modelBuffer, options);
//        } catch (IOException e) {
//            throw new IOException("BODY access failed", e);
//        }
//
//        warmUp();
    }

    private void warmUp() {
        float[][] dummyObs0 = new float[1][10];
        float[][] dummyObs1 = new float[1][8];
        float[][] outDeterministic = new float[1][2];

        Object[] inputsArr = new Object[]{dummyObs0, dummyObs1};
        Map<Integer, Object> outputsMap = new HashMap<>();
        outputsMap.put(0, outDeterministic); // if wrong output, try index 1
        tflite.runForMultipleInputsOutputs(inputsArr, outputsMap);
    }

    public float[] runDeterministic(float agentX, float agentY, float targetX, float targetY) {
        // 1) Prepare padded inputs (batch size 1)
        float[][] obs0 = new float[1][10];
        float[][] obs1 = new float[1][8];

        obs0[0][0] = agentX;
        obs0[0][1] = agentY;
        obs1[0][0] = targetX;
        obs1[0][1] = targetY;

        // 2) Safe preprocessing: protect against zero std
        final float EPS = 1e-7f;
        for (int i = 0; i < 10; i++) {
            float std = OBS0_STD[i];
            if (Math.abs(std) < EPS) std = EPS;          // avoid divide-by-zero
            float v = (obs0[0][i] - OBS0_MEAN[i]) / std;
            obs0[0][i] = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
        }
        for (int i = 0; i < 8; i++) {
            float std = OBS1_STD[i];
            if (Math.abs(std) < EPS) std = EPS;
            float v = (obs1[0][i] - OBS1_MEAN[i]) / std;
            obs1[0][i] = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
        }

        // 3) Output container (must be allocated)
        float[][] identity2 = new float[1][2];

        Object[] inputsArr = new Object[]{obs0, obs1};

        // 4) Try index-based outputs (0 then 1). Provide informative exception on failure.
        RuntimeException lastException = null;
        // helper to validate populated outputs
        java.util.function.Predicate<float[][]> validOutput = out ->
                out != null && out.length >= 1 && out[0] != null && out[0].length >= 2
                        && Float.isFinite(out[0][0]) && Float.isFinite(out[0][1]);

        // Try index 0
        try {
            java.util.Map<Integer, Object> outputs0 = new java.util.HashMap<>();
            outputs0.put(0, identity2);
            tflite.runForMultipleInputsOutputs(inputsArr, outputs0);

            if (validOutput.test(identity2)) {
                return new float[]{ identity2[0][0], identity2[0][1] };
            } else {
                throw new RuntimeException("Output at index 0 invalid (null/NaN).");
            }
        } catch (RuntimeException e0) {
            lastException = e0;
            // fall through to try index 1
        }

        // Reset container
        identity2[0][0] = 0f;
        identity2[0][1] = 0f;

        // Try index 1
        try {
            java.util.Map<Integer, Object> outputs1 = new java.util.HashMap<>();
            outputs1.put(1, identity2);
            tflite.runForMultipleInputsOutputs(inputsArr, outputs1);

            if (validOutput.test(identity2)) {
                return new float[]{ identity2[0][0], identity2[0][1] };
            } else {
                throw new RuntimeException("Output at index 1 invalid (null/NaN).");
            }
        } catch (RuntimeException e1) {
            String msg = "TFLite inference failed for both output indices. last: "
                    + (e1.getMessage() == null ? e1.toString() : e1.getMessage())
                    + " ; earlier: " + (lastException != null ? lastException.getMessage() : "none");
            // throw a runtime exception so caller (OpMode) can telemetry/log it
            throw new RuntimeException(msg, e1);
        }

//        return new float[]{ identity2[0][0], identity2[0][1]};
    }


    public void close() {
        tflite.close();
    }
}
