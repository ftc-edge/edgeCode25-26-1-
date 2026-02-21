/*
* CREATED BY TEAM 22346 EDGE
* A Cameron Scullion Project
* YAIBA and all surrounding content are entirely free use for all FTC teams
* */

package org.firstinspires.ftc.teamcode.yaiba;

import ai.onnxruntime.*;
import android.content.res.AssetManager;
import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.nio.FloatBuffer;
import java.util.Collections;

public class YAIBAONNX {
    private static final String TAG = "YAIBAONNX";
    private static final String MODEL_FILENAME = "YAIBA.onnx";
    private OrtEnvironment env;
    private OrtSession session;

    private static final int NUM_OBSERVATIONS = 9;
    private static final int NUM_ACTIONS = 3;

    public YAIBAONNX(AssetManager assetManager) throws OrtException {
        // Initialize ONNX Runtime environment
        env = OrtEnvironment.getEnvironment();

        ensureSession(assetManager);
    }

    /**
     * Run inference on observations
     * @param observations Array of 9 floats matching observation order
     * @return Array of 3 floats [strafe, forward, rotation]
     */

    //This gets run in your OpMode. Use YAIBAONNX.predict
    public float[] predict(float[] observations, AssetManager assetManager) throws OrtException {
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
     * Clean up resources
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
    }
}