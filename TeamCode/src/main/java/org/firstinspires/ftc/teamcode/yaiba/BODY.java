package org.firstinspires.ftc.teamcode.yaiba;

import android.content.Context;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

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
        MappedByteBuffer modelBuffer = FileUtil.loadMappedFile(context, "yaiba.tflite");
        Interpreter.Options options = new Interpreter.Options();
        options.setNumThreads(2);
        options.setUseNNAPI(false);
        tflite = new Interpreter(modelBuffer, options);

        warmUp();
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
        float[][] obs0 = new float[1][10];
        float[][] obs1 = new float[1][8];

        obs0[0][0] = agentX;
        obs0[0][1] = agentY;
        obs1[0][0] = targetX;
        obs1[0][1] = targetY;

        for (int i = 0; i < 10; i++) {
            float v = (obs0[0][i] - OBS0_MEAN[i]) / OBS0_STD[i];
            obs0[0][i] = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
        }
        for (int i = 0; i < 8; i++) {
            float v = (obs1[0][i] - OBS1_MEAN[i]) / OBS1_STD[i];
            obs1[0][i] = Math.max(CLIP_MIN, Math.min(CLIP_MAX, v));
        }

        float[][] identity2 = new float[1][2];

        Object[] inputsArr = new Object[]{obs0, obs1};
        Map<Integer, Object> outputsMap = new HashMap<>();
        outputsMap.put(0, identity2); // deterministic action head
        tflite.runForMultipleInputsOutputs(inputsArr, outputsMap);

        return identity2[0]; // [forward, sideways], each in [-1, +1]
    }

    public void close() {
        tflite.close();
    }
}
