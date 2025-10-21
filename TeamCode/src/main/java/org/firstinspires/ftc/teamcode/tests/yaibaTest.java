//package org.firstinspires.ftc.teamcode.tests; // <- match your project package
//
//import android.util.Log;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
///**
// * YaibaDebugOpMode
// *
// * Purpose:
// *  - Help diagnose why the model output doesn't react to the robot's own position.
// *  - Telemetry/logs raw and normalized inputs that are passed to the model and the produced outputs.
// *
// * Usage:
// *  - Put BODY.tflite in app/src/main/assets/
// *  - Use TensorInspector (optional) to confirm tensor shapes first.
// *  - Run this OpMode. Observe Driver Station telemetry and logcat (tag = "YaibaDebug").
// *
// * Controls:
// *  - Set DEBUG_SYNTHETIC_MODE = true to run a synthetic sweep test of the first observation index.
// */
//@TeleOp(name = "Yaiba: Debugger", group = "Debug")
//public class YaibaDebugOpMode extends LinearOpMode {
//    private static final String TAG = "YaibaDebug";
//
//    // Toggle this to run synthetic tests instead of live sensor reads.
//    // Synthetic mode cycles the first observation feature so you can see if the model output changes.
//    private static final boolean DEBUG_SYNTHETIC_MODE = false;
//
//    private BODY yaiba = null;
//
//    @Override
//    public void runOpMode() {
//        telemetry.addLine("Yaiba Debug OpMode - initializing...");
//        telemetry.update();
//
//        // Create model safely (factory returns null on error).
//        yaiba = BODY.create(hardwareMap.appContext);
//        if (yaiba == null) {
//            telemetry.addLine("ERROR: failed to load model (BODY.tflite)");
//            telemetry.addLine("Check app/src/main/assets/BODY.tflite");
//            telemetry.update();
//            Log.e(TAG, "BODY.create returned null - cannot continue inference.");
//            // We still continue so the user can observe that model is missing.
//        } else {
//            telemetry.addLine("Model loaded. Dumping model IO to log for debugging...");
//            telemetry.update();
//            // If BODY has dumpModelIoInfo() method (the version I gave does), call it to log shapes.
//            // This helps confirm input/output tensor shapes and ordering.
//            try {
//                // reflection safe-call (in case your BODY doesn't expose it publicly)
//                yaiba.getClass().getDeclaredMethod("dumpModelIoInfo").setAccessible(true);
//                yaiba.getClass().getDeclaredMethod("dumpModelIoInfo").invoke(yaiba);
//            } catch (NoSuchMethodException nsme) {
//                Log.i(TAG, "dumpModelIoInfo() not available on BODY; ignore.");
//            } catch (Exception e) {
//                Log.w(TAG, "Error calling dumpModelIoInfo(): " + e.toString(), e);
//            }
//        }
//
//        waitForStart();
//
//        // Example loop variables
//        long loopIndex = 0;
//        float syntheticValue = -2.0f;
//        float syntheticStep = 0.25f;
//        boolean syntheticIncreasing = true;
//
//        while (opModeIsActive()) {
//            loopIndex++;
//
//            // ---------- 1) Gather raw observations ----------
//            // Replace this block with your actual sensors/localization values.
//            // The variable names below are example mappings; adapt to your sensors.
//            float raw_agentX;
//            float raw_agentY;
//            float raw_targetX;
//            float raw_targetY;
//
//            if (DEBUG_SYNTHETIC_MODE) {
//                // synthetic test: vary only the first feature (agentX) over time
//                raw_agentX = syntheticValue;
//                raw_agentY = 0f;
//                raw_targetX = 1f;
//                raw_targetY = 0f;
//
//                if (syntheticIncreasing) syntheticValue += syntheticStep;
//                else syntheticValue -= syntheticStep;
//                if (syntheticValue > 2.0f) syntheticIncreasing = false;
//                if (syntheticValue < -2.0f) syntheticIncreasing = true;
//            } else {
//                // LIVE mode: read actual sensors / odometry here.
//                // Replace the following placeholders with your actual readings.
//                // Example: raw_agentX = odometry.getX();
//                // Example: raw_agentY = odometry.getY();
//                // Example: raw_targetX = vision.getTargetX();
//                // Example: raw_targetY = vision.getTargetY();
//                raw_agentX = getRobotX();  // <-- implement these helper functions below
//                raw_agentY = getRobotY();
//                raw_targetX = getTargetX();
//                raw_targetY = getTargetY();
//            }
//
//            // ---------- 2) Build the exact observation arrays we will feed the model ----------
//            // IMPORTANT: adjust sizes/indexes to match your actual model layout!
//            // We'll assume the model expects two inputs: obs0 shape [1,10] and obs1 shape [1,8]
//            // If your model is different, change the array sizes accordingly.
//            float[][] obs0 = new float[1][10];
//            float[][] obs1 = new float[1][8];
//
//            // Fill obs0/obs1 with the raw values in the mapping you believe matches training.
//            // This is where mistakes are commonly made: the order must match training exactly.
//            // Example mapping (change to your model's mapping):
//            obs0[0][0] = raw_agentX;   // candidate index for agentX
//            obs0[0][1] = raw_agentY;   // candidate index for agentY
//            obs0[0][2] = raw_targetX;  // candidate index for targetX
//            obs0[0][3] = raw_targetY;  // candidate index for targetY
//            // The rest are placeholders (zeros) — fill them with real features if your model needs them
//            for (int i = 4; i < obs0[0].length; i++) obs0[0][i] = 0f;
//
//            // Example obs1 - placeholders
//            for (int i = 0; i < obs1[0].length; i++) obs1[0][i] = 0f;
//
//            // ---------- 3) Normalize the observations (if your model used normalization) ----------
//            // If your model expects normalized inputs (means/stds used in training), do it here.
//            // If you don't know the normalization, leave this as an identity transform and test.
//            float[][] normObs0 = new float[1][obs0[0].length];
//            float[][] normObs1 = new float[1][obs1[0].length];
//
//            // Example normalization placeholders: replace with training means/stds if known.
//            float[] obs0_mean = new float[]{0f,0f,0f,0f,0f,0f,0f,0f,0f,0f};
//            float[] obs0_std  = new float[]{1f,1f,1f,1f,1f,1f,1f,1f,1f,1f};
//            float[] obs1_mean = new float[]{0f,0f,0f,0f,0f,0f,0f,0f};
//            float[] obs1_std  = new float[]{1f,1f,1f,1f,1f,1f,1f,1f};
//
//            for (int i = 0; i < obs0[0].length; i++) {
//                float v = obs0[0][i];
//                float m = (i < obs0_mean.length) ? obs0_mean[i] : 0f;
//                float s = (i < obs0_std.length && obs0_std[i] != 0f) ? obs0_std[i] : 1f;
//                normObs0[0][i] = (v - m) / s;
//            }
//            for (int i = 0; i < obs1[0].length; i++) {
//                float v = obs1[0][i];
//                float m = (i < obs1_mean.length) ? obs1_mean[i] : 0f;
//                float s = (i < obs1_std.length && obs1_std[i] != 0f) ? obs1_std[i] : 1f;
//                normObs1[0][i] = (v - m) / s;
//            }
//
//            // ---------- 4) Run inference safely, capturing outputs ----------
//            float[] action = new float[]{0f, 0f}; // default fallback
//            if (yaiba != null) {
//                try {
//                    Object[] inputs = new Object[]{ normObs0, normObs1 };
//
//                    // Prepare output holder. Update dimensions to match your model's output shape.
//                    float[][] outBuf = new float[1][2]; // suppose output is [1,2]
//                    java.util.Map<Integer, Object> outputs = new java.util.HashMap<>();
//                    outputs.put(0, outBuf); // output index 0
//
//                    actions = yaiba.runDeterministic(raw_agentX,  raw_agentY, raw_targetX, raw_targetY); // <--- see note below
//                    // If your BODY doesn't expose runForMultipleInputsOutputsSafe, call its inference method.
//                    // We assume here that BODY has a method that wraps runForMultipleInputsOutputs, or
//                    // you can instead call yaiba.runDeterministic(...) if you implemented it.
//
//                    action[0] = outBuf[0][0];
//                    action[1] = outBuf[0][1];
//                } catch (Exception e) {
//                    // Catch any inference-time exceptions and log them
//                    Log.e(TAG, "Inference error: " + e.toString(), e);
//                    telemetry.addData("InferenceError", e.toString());
//                }
//            } else {
//                telemetry.addData("Model", "Not loaded");
//            }
//
//            // ---------- 5) Telemetry + Log everything (raw -> normalized -> output) ----------
//            telemetry.clearAll();
//            telemetry.addData("loop", loopIndex);
//            telemetry.addData("RAW OBS (first 4) : aX=%.3f aY=%.3f tX=%.3f tY=%.3f", "smth", raw_agentX, raw_agentY, raw_targetX, raw_targetY);
//            // Print first obs0/obs1 elements (adjust as needed)
//            telemetry.addData("OBS0[0..5] raw: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", "smth", obs0[0][0], obs0[0][1], obs0[0][2], obs0[0][3], obs0[0][4], obs0[0][5]);
//            telemetry.addData("OBS0[0..5] norm: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", "smth", normObs0[0][0], normObs0[0][1], normObs0[0][2], normObs0[0][3], normObs0[0][4], normObs0[0][5]);
//            telemetry.addData("ACTION out: %.4f, %.4f", "smth", action[0], action[1]);
//            telemetry.update();
//
//            // Also log to Android log for easier search (adb logcat -s YaibaDebug)
//            Log.i(TAG, String.format("loop=%d RAW aX=%.3f aY=%.3f tX=%.3f tY=%.3f | ACTION=%.4f,%.4f",
//                    loopIndex, raw_agentX, raw_agentY, raw_targetX, raw_targetY, action[0], action[1]));
//
//            // ---------- 6) Optionally apply the action to robot here (commented out) ----------
//            // applyActionToMotors(action);
//
//            // modest sleep so log/telemetry are readable
//            try { Thread.sleep(150); } catch (InterruptedException e) { Thread.currentThread().interrupt(); }
//        }
//
//        // Close model ONCE after OpMode ends
//        if (yaiba != null) {
//            yaiba.close();
//            yaiba = null;
//        }
//    }
//
//    // ---------- Helper placeholder implementations ----------
//    // Replace these with real odometry/vision calls. For now they return 0 so you must update them.
//    private float getRobotX() {
//        // e.g., return (float) odometry.getPose().getX();
//        return 0f;
//    }
//    private float getRobotY() {
//        // e.g., return (float) odometry.getPose().getY();
//        return 0f;
//    }
//    private float getTargetX() {
//        // e.g., vision pipeline target center X (meters or normalized)
//        return 1f;
//    }
//    private float getTargetY() {
//        return 0f;
//    }
//}
