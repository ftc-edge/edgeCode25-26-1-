package org.firstinspires.ftc.teamcode.yaiba;

import ai.onnxruntime.*;
import android.content.res.AssetManager;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

public class BODYONNX{
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

            // Create session
            OrtSession.SessionOptions opts = new OrtSession.SessionOptions();
            session = env.createSession(modelBytes, opts);

            // Optional: Print model info for debugging
            session.getInputNames().forEach(name ->
                    System.out.println("Input: " + name));
            session.getOutputNames().forEach(name ->
                    System.out.println("Output: " + name));

        } catch (Exception e) {
            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
                    "Failed to load model: " + e.getMessage());
        }
    }

    /**
     * Run inference on observations
     * @param observations Array of 9 floats matching Unity observation order
     * @return Array of 3 floats [strafe, forward, rotation]
     */
    public float[] predict(float[] observations) throws OrtException {
        if (observations.length != NUM_OBSERVATIONS) {
            throw new IllegalArgumentException(
                    "Expected " + NUM_OBSERVATIONS + " observations, got " + observations.length);
        }

        // Create input tensor
        // Shape: [1, 9] - batch size of 1, 9 observations
        long[] shape = {1, NUM_OBSERVATIONS};
        OnnxTensor inputTensor = OnnxTensor.createTensor(env,
                new float[][]{observations});

        // Run inference
        // ML-Agents exports use specific input/output names
        Map<String, OnnxTensor> inputs = new HashMap<>();
        inputs.put("obs_0", inputTensor);  // Default ML-Agents input name

        try (OrtSession.Result results = session.run(inputs)) {
            // Get continuous actions output
            // ML-Agents typically names this "continuous_actions"
            float[][] output = (float[][]) results.get("continuous_actions")
                    .get()
                    .getValue();

            inputTensor.close();

            // Return the action array [strafe, forward, rotation]
            return output[0];

        } catch (Exception e) {
            inputTensor.close();
            throw new OrtException(OrtException.OrtErrorCode.ORT_FAIL,
                    "Inference failed: " + e.getMessage());
        }
    }

    /**
     * Clean up resources
     */
    public void close() throws OrtException {
        if (session != null) {
            session.close();
        }
        // Note: Don't close env as it's a singleton
    }
}