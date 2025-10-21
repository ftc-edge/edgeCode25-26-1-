package org.firstinspires.ftc.teamcode.yaiba; // change to match your package

import android.content.Context;
import android.util.Log;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.Interpreter.Options;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;
import java.nio.MappedByteBuffer;
import java.util.HashMap;
import java.util.Map;

/**
 * BODY - lightweight TFLite wrapper for YAIBA agent.
 *
 * Usage (safe):
 *   BODY yaiba = BODY.create(hardwareMap.appContext);
 *   if (yaiba == null) {
 *       // model failed to load; handle gracefully
 *   } else {
 *       float[] action = yaiba.runDeterministic(agentX, agentY, targetX, targetY);
 *       yaiba.close();
 *   }
 *
 * Notes:
 *  - Place BODY.tflite in app/src/main/assets/
 *  - Do NOT include tensorflow-lite-select-tf-ops if your model doesn't require it.
 *  - If you prefer the original behavior, the constructor still exists and throws IOException.
 */
public class BODY implements AutoCloseable {
    private static final String TAG = "BODY";
    private static final String MODEL_FILENAME = "BODY.tflite"; // placed in src/main/assets/

    // Example normalization arrays — REPLACE with exact values used at training time.
    // Update lengths to match model input sizes.
    private static final float[] OBS0_MEAN = new float[]{0f,0f,0f,0f,0f,0f,0f,0f,0f,0f}; // example length 10
    private static final float[] OBS0_STD  = new float[]{1f,1f,1f,1f,1f,1f,1f,1f,1f,1f};
    private static final float[] OBS1_MEAN = new float[]{0f,0f,0f,0f,0f,0f,0f,0f}; // example length 8
    private static final float[] OBS1_STD  = new float[]{1f,1f,1f,1f,1f,1f,1f,1f};

    private static final float CLIP_MIN = -5.0f;
    private static final float CLIP_MAX = 5.0f;

    // TFLite interpreter
    private Interpreter tflite = null;

    /**
     * Original constructor: loads model and may throw IOException on failure.
     * Keeping this so callers that want the exception can use it.
     */
    public BODY(Context context) throws IOException {
        if (context == null) throw new IllegalArgumentException("Context is null");

        // Load TFLite model file as memory-mapped buffer
        MappedByteBuffer modelBuffer = FileUtil.loadMappedFile(context, MODEL_FILENAME);

        // Interpreter options: avoid NNAPI and multi-thread surprises on control hub
        Options options = new Options();
        options.setNumThreads(1);
        options.setUseNNAPI(false);
        try {
            tflite = new Interpreter(modelBuffer, options);
            Log.i(TAG, "TFLite Interpreter created (builtins-only).");
        } catch (Exception e) {
            Log.e(TAG, "Failed to create TFLite Interpreter: " + e.toString(), e);
            // Rethrow as IOException so callers can catch if they used 'new BODY(...)'
            throw new IOException("Failed to initialize TFLite Interpreter: " + e.getMessage(), e);
        }
    }

    /**
     * Factory method that wraps the constructor and returns null on failure.
     * This is convenient for OpMode initialization where you want to avoid
     * checked exceptions but still be able to handle startup failure gracefully.
     */
    public static BODY create(Context context) {
        try {
            return new BODY(context);
        } catch (IOException e) {
            Log.e(TAG, "BODY.create: failed to initialize: " + e.toString(), e);
            return null;
        }
    }

    /**
     * Run deterministic policy: returns action [forward/backward, horizontal].
     * Inputs are raw scalar values (agentX, agentY, targetX, targetY) — adapt if you pass different obs.
     *
     * NOTE: This method normalizes/clips using OBS*_MEAN/OBS*_STD and CLIP_MIN/MAX arrays above.
     * Make sure these values match the ones used at training time.
     */
    public float[] runDeterministic(float agentX, float agentY, float targetX, float targetY) {
        if (tflite == null) {
            Log.e(TAG, "Interpreter not initialized (runDeterministic). Returning zeros.");
            return new float[]{0f, 0f};
        }

        // Construct observations arrays expected by the model.
        // The shapes below assume model expects obs0 shape [1,10] and obs1 shape [1,8].
        // Replace or adapt to your model's expected shapes & values.
        float[][] obs0 = new float[1][10];
        float[][] obs1 = new float[1][8];

        // Example population: adapt to your actual observation layout
        obs0[0][0] = agentX;
        obs0[0][1] = agentY;
        obs1[0][0] = targetX;
        obs1[0][1] = targetY;
        // the remainder of obs0/obs1 should be filled per your model's expectation (zeros as placeholder)
        for (int i = 4; i < obs0[0].length; i++) obs0[0][i] = 0f;
        for (int i = 0; i < obs1[0].length; i++) obs1[0][i] = 0f;

        // Normalize and clip obs0
        for (int i = 0; i < obs0[0].length && i < OBS0_MEAN.length && i < OBS0_STD.length; i++) {
            float v = (obs0[0][i] - OBS0_MEAN[i]) / OBS0_STD[i];
            v = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
            obs0[0][i] = v;
        }

        // Normalize and clip obs1
        for (int i = 0; i < obs1[0].length && i < OBS1_MEAN.length && i < OBS1_STD.length; i++) {
            float v = (obs1[0][i] - OBS1_MEAN[i]) / OBS1_STD[i];
            v = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
            obs1[0][i] = v;
        }

        // Prepare outputs - ensure this matches your model's output shape
        float[][] identity2 = new float[1][2]; // expected deterministic action head [1,2]

        // Run inference using runForMultipleInputsOutputs
        Object[] inputsArr = new Object[]{obs0, obs1};
        Map<Integer, Object> outputsMap = new HashMap<>();
        outputsMap.put(0, identity2); // ensure this index matches the model's output order

        try {
            tflite.runForMultipleInputsOutputs(inputsArr, outputsMap);
        } catch (IllegalArgumentException iae) {
            Log.e(TAG, "runForMultipleInputsOutputs threw IllegalArgumentException: " + iae.toString(), iae);
            // Log model IO info for debugging
            dumpModelIoInfo();
            return new float[]{0f, 0f};
        } catch (Exception e) {
            Log.e(TAG, "Inference failed: " + e.toString(), e);
            dumpModelIoInfo();
            return new float[]{0f, 0f};
        }

        // Postprocess: identity2[0] contains the deterministic action
        float[] out = new float[]{identity2[0][0], identity2[0][1]};
        return out;
    }

    /**
     * Helpful debug method: prints model input/output info to the log when things fail.
     * You can extend this to dump tensor details (shapes, dtypes) if needed.
     */
    private void dumpModelIoInfo() {
        if (tflite == null) {
            Log.i(TAG, "dumpModelIoInfo: interpreter is null.");
            return;
        }
        try {
            int inputCount = tflite.getInputTensorCount();
            int outputCount = tflite.getOutputTensorCount();
            Log.i(TAG, "Model IO: inputCount=" + inputCount + " outputCount=" + outputCount);
            for (int i = 0; i < inputCount; i++) {
                try {
                    Log.i(TAG, " Input[" + i + "]: name=" + tflite.getInputTensor(i).name()
                            + " shape=" + java.util.Arrays.toString(tflite.getInputTensor(i).shape()));
                } catch (Exception ex) {
                    Log.w(TAG, " Could not read input tensor " + i + ": " + ex.toString());
                }
            }
            for (int i = 0; i < outputCount; i++) {
                try {
                    Log.i(TAG, " Output[" + i + "]: name=" + tflite.getOutputTensor(i).name()
                            + " shape=" + java.util.Arrays.toString(tflite.getOutputTensor(i).shape()));
                } catch (Exception ex) {
                    Log.w(TAG, " Could not read output tensor " + i + ": " + ex.toString());
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "dumpModelIoInfo failed: " + e.toString(), e);
        }
    }

    @Override
    public void close() {
        if (tflite != null) {
            try {
                tflite.close();
                Log.i(TAG, "Interpreter closed");
            } catch (Exception e) {
                Log.w(TAG, "Error closing interpreter: " + e.toString());
            } finally {
                tflite = null;
            }
        }
    }
}
