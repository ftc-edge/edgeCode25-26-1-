package org.firstinspires.ftc.teamcode.yaiba;

import ai.onnxruntime.*;
import android.content.res.AssetManager;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class BODYONNX {
    private static final String TAG = "BODYONNX";
    private static final String MODEL_FILENAME = "BODY.onnx";
    private OrtEnvironment env;
    private OrtSession session;

    private static final int NUM_OBSERVATIONS = 9;
    private static final int NUM_ACTIONS = 3;

    public BODYONNX(AssetManager assetManager) throws OrtException {
        // Initialize ONNX Runtime environment
        env = OrtEnvironment.getEnvironment();

        ensureSession(assetManager);
    }

    /**
     * Run inference - matching TFLite's method signature
     */

    /**
     * Run inference on observations
     * @param observations Array of 9 floats matching Unity observation order
     * @return Array of 3 floats [strafe, forward, rotation] - ALWAYS A NEW ARRAY
     */
    public float[] predict(float[] observations, AssetManager assetManager) throws OrtException {
//        if (observations.length != NUM_OBSERVATIONS) {
//            throw new IllegalArgumentException(
//                    "Expected " + NUM_OBSERVATIONS + " observations, got " + observations.length);
//        }
//
//        long[] shape = {1, NUM_OBSERVATIONS};
//
//        float[][] inputArray = new float[1][NUM_OBSERVATIONS];
//        System.arraycopy(observations, 0, inputArray[0], 0, NUM_OBSERVATIONS);
//
//        OnnxTensor inputTensor = null;
//
//        try {
//            inputTensor = OnnxTensor.createTensor(env, inputArray);
//
//            Map<String, OnnxTensor> inputs = new HashMap<>();
//            inputs.put("obs_0", inputTensor);
//
//            OrtSession.Result results = session.run(inputs);
//
//            // DEBUG: Print all output names
//            Log.i(TAG, "Available outputs: " + results.size());
//            for (int i = 0; i < results.size(); i++) {
//                try {
//                    OnnxValue val = results.get(i);
//                    Log.i(TAG, "Output " + i + ": shape=" + Arrays.toString((long[]) (((float[][])val.getValue()).length > 0 ?
//                                                ((float[][])val.getValue())[0] : "scalar")));
//                } catch (Exception e) {
//                    Log.i(TAG, "Output " + i + ": " + e.getMessage());
//                }
//            }
//
//            OnnxValue outputValue = results.get(4);  // Changed from get("continuous_actions")
//            float[][] outputArray = (float[][]) outputValue.getValue();
//
//            Log.i(TAG, "Using output[2]: " + Arrays.toString(outputArray[0]));
//
//            // Extract actions
//            float[] actions = new float[NUM_ACTIONS];
//            actions[0] = outputArray[0][0];  // strafe
//            actions[1] = outputArray[0][1];  // forward
//            actions[2] = outputArray[0][2];  // rotation
//
//            results.close();
//            inputTensor.close();
//
//            // Clamp outputs
//            for (int i = 0; i < NUM_ACTIONS; i++) {
//                if (Float.isNaN(actions[i]) || Float.isInfinite(actions[i])) {
//                    actions[i] = 0.0f;
//                }
//                actions[i] = Math.max(-1.0f, Math.min(1.0f, actions[i]));
//            }
//
//            return actions;
//
//        } catch (Exception e) {
//            Log.e(TAG, "Inference failed: " + e.getMessage(), e);
//            if (inputTensor != null) {
//                try {
//                    inputTensor.close();
//                } catch (Exception ex) {}
//            }
//            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
//                    "Inference failed: " + e.getMessage());
//        }

        ensureSession(assetManager);

        FloatBuffer buffer = FloatBuffer.wrap(observations);

        try (OnnxTensor inputTensor = OnnxTensor.createTensor(env, buffer, new long[]{1, observations.length});
             OrtSession.Result results = session.run(Collections.singletonMap("obs_0", inputTensor))) {
            return ((float[][]) results.get(2).getValue())[0];
        }
    }

    private void ensureSession(AssetManager assetManager) throws OrtException {
        if (session != null) {
            return;
        }
        if (env == null) {
            env = OrtEnvironment.getEnvironment();
        }

        try (InputStream modelStream = assetManager.open(MODEL_FILENAME)) {
            byte[] modelBytes = new byte[modelStream.available()];
            int read = modelStream.read(modelBytes);
            if (read <= 0) {
                throw new IOException("Model asset is empty: " + MODEL_FILENAME);
            }

            OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
            opts.setOptimizationLevel(OrtSession.SessionOptions.OptLevel.ALL_OPT);
            opts.setInterOpNumThreads(1);
            opts.setIntraOpNumThreads(1);

            session = env.createSession(modelBytes, opts);

            Log.i(TAG, "ONNX Model loaded successfully: " + MODEL_FILENAME);
            session.getInputNames().forEach(name -> Log.i(TAG, "Input: " + name));
            session.getOutputNames().forEach(name -> Log.i(TAG, "Output: " + name));
        } catch (Exception e) {
            Log.e(TAG, "Failed to load model: " + e.getMessage(), e);
            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
                    "Failed to load model: " + e.getMessage());
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