package org.firstinspires.ftc.teamcode.tests; // <- change to match your package

import android.content.Context;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.io.IOException;
import java.lang.reflect.Method;
import java.nio.MappedByteBuffer;
import java.util.Set;

/**
 * TensorInspector - load a .tflite file and log input/output tensor names, shapes, and dtypes.
 *
 * This version is defensive:
 *  - It uses reflection to attempt to access newer Interpreter APIs (getSignatureKeys/getSignatureRunner)
 *    without crashing on TF Lite versions that don't have them.
 *
 * Usage:
 *   TensorInspector.inspectModel(hardwareMap.appContext, "BODY.tflite");
 *
 * Make sure the model file exists in app/src/main/assets/
 */


public class TensorInspector {
    private static final String TAG = "TensorInspector";

    /**
     * Loads model and logs detailed tensor info.
     *
     * @param context   Android context (hardwareMap.appContext)
     * @param assetName file name in assets (e.g., "BODY.tflite")
     */
    public static void inspectModel(Context context, String assetName) {
        if (context == null) {
            Log.e(TAG, "inspectModel: context is null");
            return;
        }

        try {
            MappedByteBuffer modelBuf = FileUtil.loadMappedFile(context, assetName);
            Interpreter.Options opts = new Interpreter.Options();
            opts.setNumThreads(1);
            Interpreter interpreter = new Interpreter(modelBuf, opts);

            int inputCount = interpreter.getInputTensorCount();
            int outputCount = interpreter.getOutputTensorCount();
            Log.i(TAG, "Model loaded: " + assetName + "  inputCount=" + inputCount + " outputCount=" + outputCount);

            // Inputs
            for (int i = 0; i < inputCount; i++) {
                try {
                    org.tensorflow.lite.Tensor tensor = interpreter.getInputTensor(i);
                    String name = tensor.name();
                    int[] shape = tensor.shape();
                    org.tensorflow.lite.DataType dtype = tensor.dataType();
                    Log.i(TAG, " Input[" + i + "]: name=\"" + name + "\" dtype=" + dtype + " shape=" + java.util.Arrays.toString(shape));
                } catch (Exception ex) {
                    Log.e(TAG, " Error reading input tensor " + i + ": " + ex.toString(), ex);
                }
            }

            // Outputs
            for (int i = 0; i < outputCount; i++) {
                try {
                    org.tensorflow.lite.Tensor tensor = interpreter.getOutputTensor(i);
                    String name = tensor.name();
                    int[] shape = tensor.shape();
                    org.tensorflow.lite.DataType dtype = tensor.dataType();
                    Log.i(TAG, " Output[" + i + "]: name=\"" + name + "\" dtype=" + dtype + " shape=" + java.util.Arrays.toString(shape));
                } catch (Exception ex) {
                    Log.e(TAG, " Error reading output tensor " + i + ": " + ex.toString(), ex);
                }
            }

            // Try to read signature keys reflectively (safe on versions without the API)
            try {
                Method getSigKeys = interpreter.getClass().getMethod("getSignatureKeys");
                Object sigKeysObj = getSigKeys.invoke(interpreter);
                if (sigKeysObj instanceof Set) {
                    @SuppressWarnings("unchecked")
                    Set<String> sigKeys = (Set<String>) sigKeysObj;
                    Log.i(TAG, "Signature keys (via getSignatureKeys): " + sigKeys);
                } else {
                    Log.i(TAG, "getSignatureKeys returned non-Set: " + sigKeysObj);
                }
            } catch (NoSuchMethodException nsme) {
                Log.i(TAG, "getSignatureKeys() not available on this TF Lite runtime.");
            } catch (Exception e) {
                Log.w(TAG, "Error invoking getSignatureKeys(): " + e.toString(), e);
            }

            // Try to detect available signature runners (reflectively list known runners)
            // This is just an exploratory attempt: getSignatureRunner(String) exists on some builds.
            try {
                // Some interpreters expose getSignatureRunner(String). We attempt to call it for each key if we have keys.
                Method getSigKeys = null;
                try {
                    getSigKeys = interpreter.getClass().getMethod("getSignatureKeys");
                } catch (NoSuchMethodException ignore) { /* not available */ }

                if (getSigKeys != null) {
                    Object sigObj = getSigKeys.invoke(interpreter);
                    if (sigObj instanceof Set) {
                        @SuppressWarnings("unchecked")
                        Set<String> keys = (Set<String>) sigObj;
                        for (String key : keys) {
                            Log.i(TAG, "Attempting to probe signature runner for key: " + key);
                            try {
                                Method getRunner = interpreter.getClass().getMethod("getSignatureRunner", String.class);
                                Object runner = getRunner.invoke(interpreter, key);
                                if (runner != null) {
                                    Log.i(TAG, " SignatureRunner for key \"" + key + "\" is available (object: " + runner.getClass().getName() + ")");
                                } else {
                                    Log.i(TAG, " SignatureRunner for key \"" + key + "\" returned null.");
                                }
                            } catch (NoSuchMethodException nsme2) {
                                Log.i(TAG, "getSignatureRunner(String) not available on this TF Lite runtime.");
                                break;
                            } catch (Exception exRunner) {
                                Log.w(TAG, "Error probing SignatureRunner for key " + key + ": " + exRunner.toString(), exRunner);
                            }
                        }
                    }
                } else {
                    Log.i(TAG, "Signature runner probing skipped (getSignatureKeys not available).");
                }
            } catch (Exception e) {
                Log.w(TAG, "Signature runner probing failed: " + e.toString(), e);
            }

            interpreter.close();
            Log.i(TAG, "Model inspection complete.");
        } catch (IOException e) {
            Log.e(TAG, "inspectModel failed to load asset \"" + assetName + "\": " + e.toString(), e);
        } catch (Exception e) {
            Log.e(TAG, "inspectModel unexpected error: " + e.toString(), e);
        }
    }
}
