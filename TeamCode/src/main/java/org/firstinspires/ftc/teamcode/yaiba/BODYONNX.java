package org.firstinspires.ftc.teamcode.yaiba;

import ai.onnxruntime.*;
import android.content.res.AssetManager;
import android.util.Log;
import java.io.InputStream;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

public class BODYONNX {
    private static final String TAG = "BODYONNX";
    private OrtEnvironment env;
    private OrtSession session;

    private static final int NUM_OBSERVATIONS = 9;
    private static final int NUM_ACTIONS = 3;

    public BODYONNX(AssetManager assetManager) throws OrtException {
        // Initialize ONNX Runtime environment
        env = OrtEnvironment.getEnvironment();

        try {
            // Load model from assets
            InputStream modelStream = assetManager.open("BODY.onnx");
            byte[] modelBytes = new byte[modelStream.available()];
            modelStream.read(modelBytes);
            modelStream.close();

            // Create session with optimization
            OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
            opts.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
            opts.setInterOpNumThreads(1);  // Match TFLite's single-thread behavior
            opts.setIntraOpNumThreads(1);

            session = env.createSession(modelBytes, opts);

            // Debug: Print model info
            Log.i(TAG, "ONNX Model loaded successfully");
            session.getInputNames().forEach(name ->
                    Log.i(TAG, "Input: " + name));
            session.getOutputNames().forEach(name ->
                    Log.i(TAG, "Output: " + name));

        } catch (Exception e) {
            Log.e(TAG, "Failed to load model: " + e.getMessage(), e);
            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
                    "Failed to load model: " + e.getMessage());
        }
    }

    /**
     * Run inference - matching TFLite's method signature
     */
    public float[] runDeterministic(float relX, float relY, float currentSin, float currentCos,
                                    float desiredSin, float desiredCos, float stageActive,
                                    float targetStageX, float targetStageY) {
        float[] observations = new float[]{
                relX, relY, currentSin, currentCos,
                desiredSin, desiredCos, stageActive,
                targetStageX, targetStageY
        };

        try {
            return predict(observations);
        } catch (OrtException e) {
            Log.e(TAG, "Inference failed: " + e.getMessage(), e);
            return new float[]{0f, 0f, 0f};
        }
    }

    /**
     * Run inference on observations
     * @param observations Array of 9 floats matching Unity observation order
     * @return Array of 3 floats [strafe, forward, rotation] - ALWAYS A NEW ARRAY
     */
    public float[] predict(float[] observations) throws OrtException {
        if (observations.length != NUM_OBSERVATIONS) {
            throw new IllegalArgumentException(
                    "Expected " + NUM_OBSERVATIONS + " observations, got " + observations.length);
        }

        // Force log EVERY inference
        System.out.println("========== PREDICT CALLED ==========");
        System.out.println("Input: " + Arrays.toString(observations));

        long[] shape = {1, NUM_OBSERVATIONS};

        float[][] inputArray = new float[1][NUM_OBSERVATIONS];
        System.arraycopy(observations, 0, inputArray[0], 0, NUM_OBSERVATIONS);

        OnnxTensor inputTensor = null;

        try {
            inputTensor = OnnxTensor.createTensor(env, inputArray);

            System.out.println("Input tensor created");

            Map<String, OnnxTensor> inputs = new HashMap<>();
            inputs.put("obs_0", inputTensor);

            System.out.println("About to run inference...");
            long startTime = System.nanoTime();

            OrtSession.Result results = session.run(inputs);

            long inferenceTime = (System.nanoTime() - startTime) / 1_000_000;
            System.out.println("Inference completed in " + inferenceTime + "ms");

            OnnxValue outputValue = results.get("continuous_actions").get();
            float[][] outputArray = (float[][]) outputValue.getValue();

            System.out.println("Raw ONNX output: " + Arrays.deepToString(outputArray));

            // Create BRAND NEW array with NEW object
            float[] actions = new float[NUM_ACTIONS];
            actions[0] = outputArray[0][0];
            actions[1] = outputArray[0][1];
            actions[2] = outputArray[0][2];

            System.out.println("Copied actions: " + Arrays.toString(actions));
            System.out.println("Action array hashcode: " + System.identityHashCode(actions));

            results.close();
            inputTensor.close();

            System.out.println("Resources closed, returning");
            System.out.println("==========================================");

            return actions;

        } catch (Exception e) {
            System.err.println("EXCEPTION in predict: " + e.getMessage());
            e.printStackTrace();
            if (inputTensor != null) {
                try {
                    inputTensor.close();
                } catch (Exception ex) {
                    // ignore
                }
            }
            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
                    "Inference failed: " + e.getMessage());
        }
    }
    /**
     * Clean up resources - matching TFLite's close() pattern
     */
    public void close() {
        if (session != null) {
            try {
                session.close();
                Log.i(TAG, "ONNX session closed");
            } catch (Exception e) {
                Log.w(TAG, "Error closing session: " + e.getMessage());
            } finally {
                session = null;
            }
        }
        // Note: Don't close env as it's a singleton
    }
}