package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.components.Constants.initHeading;
import static org.firstinspires.ftc.teamcode.components.Constants.startX;
import static org.firstinspires.ftc.teamcode.components.Constants.startY;

import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.yaiba.BODYONNX;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

import ai.onnxruntime.OrtException;

@TeleOp
public class YAIBATest extends OpMode {

    private BODYONNX model;

    private GoBildaPinpointDriver odo;

    // Scale odometry meters to Unity-style units. If center->edge reads ~10, use 0.2 to map to ~2.
    private static final double MODEL_POS_SCALE = 0.2;
    private static final double TARGET_X_M = 0.60;
    private static final double TARGET_Y_M = 0.30;

    private double robotX = 0, robotY = 0; // meters (for telemetry)
    private double targetX = TARGET_X_M;
    private double targetY = TARGET_Y_M;

    Drive drive;

    public Pose2D currentPose;
    public Pose2D startPose;

    public float targetAngle = 0f;
    public float stageX;
    public float stageY;
    boolean notStarted;

    AssetManager assetManager;

    /**
     * Build observation array matching Unity's CollectObservations
     */
    private float[] buildObservations() {
        float[] obs = new float[9];

        // Get current heading in RADIANS
        float currentHeading = (float) currentPose.getHeading(AngleUnit.RADIANS);
        robotX = currentPose.getX(DistanceUnit.CM) / 100f;
        robotY = currentPose.getY(DistanceUnit.CM) / 100f;
        // obs1/2: relative position to target, scaled for model input
        double relX = targetX - (robotX * MODEL_POS_SCALE);
        double relY = targetY - (robotY * MODEL_POS_SCALE);
        obs[0] = (float) relX;
        obs[1] = (float) relY;

        // obs3/4: sin and cos of the current angle
        obs[2] = (float)Math.sin(currentHeading);
        obs[3] = (float)Math.cos(currentHeading);

        // obs5/6: sin and cos of the desired angle
        obs[4] = (float)Math.sin(targetAngle);
        obs[5] = (float)Math.cos(targetAngle);

        //Obs 7: stage active, 1.0 if theres a intermediary stage
        obs[6] = 0.0f;

        if(obs[6] == 1){
            //obs8/9: intermediary stage location, scaled for model input
            obs[7] = (float) ((stageX - (robotX * MODEL_POS_SCALE)));
            obs[8] = (float) ((stageY - (robotY * MODEL_POS_SCALE)));
        }else{
            obs[7] = 0f;
            obs[8] = 0f;
        }

        return obs;
    }

    //update the odo pods
    private void updateOdometry() {
        odo.update();
        currentPose = odo.getPosition();
        odo.setPosition(currentPose);
    }

    @Override
    public void init() {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            drive = new Drive(hardwareMap);
            odo = hardwareMap.get(GoBildaPinpointDriver .class, "odo");
            odo.resetPosAndIMU();
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setOffsets(12, -17.5, DistanceUnit.CM);
            startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, -1.578);
            odo.setPosition(startPose);
            try {
                // Load AI model
                model = new BODYONNX(hardwareMap.appContext.getAssets());
                telemetry.addData("Status", "Model loaded successfully");
                telemetry.update();
            } catch (Exception e) {
                telemetry.addData("Error", "Failed to load model: " + e.getMessage());
                telemetry.update();
                return;
            }

        android.content.Context context = hardwareMap.appContext;
        assetManager = context.getAssets();
    }

    @Override
    public void loop() {
        if(notStarted) {
            odo.setPosition(startPose);
            notStarted = false;
        }else{
            updateOdometry();
        }

        // Build observations
        float[] observations = buildObservations();

        // CALCULATE CHECKSUM - should change every frame if observations change
        float obsChecksum = 0;
        for (float obs : observations) {
            obsChecksum += obs;
        }

        // Get AI predictions
        boolean inferenceSuccess = false;
        long inferenceTime = 0;
        long startTime = System.nanoTime();
        float[] actions = null;
        try {
            actions = model.predict(observations, assetManager);
        } catch (OrtException e) {
            throw new RuntimeException(e);
        }
        inferenceTime = (System.nanoTime() - startTime) / 1_000_000;
        inferenceSuccess = true;


        // Calculate action checksum
        float actionChecksum = actions[0] + actions[1] + actions[2];

        float strafe = actions[0];
        float forward = actions[1];
        float rotation = actions[2];

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        double heading = currentPose.getHeading(AngleUnit.RADIANS);

        // Convert meters to inches for FTC Dashboard (uses official field frame in inches)
        double robotXInches = (robotX * MODEL_POS_SCALE) * 39.3701;
        double robotYInches = (robotY * MODEL_POS_SCALE) * 39.3701;
        double targetXInches = targetX * 39.3701;
        double targetYInches = targetY * 39.3701;

        // Draw target position (red circle)
        fieldOverlay.setStroke("red");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeCircle(targetXInches, targetYInches, 4);
        fieldOverlay.setFill("red");
        fieldOverlay.setAlpha(0.3);
        fieldOverlay.fillCircle(targetXInches, targetYInches, 4);

        // Draw robot position (blue circle with direction indicator)
        fieldOverlay.setAlpha(1.0);
        fieldOverlay.setStroke("blue");
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeCircle(robotXInches, robotYInches, 9);
        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(robotXInches, robotYInches, 9);

        // Draw direction line on robot
        double lineLength = 9;
        double lineEndX = robotXInches + lineLength * Math.cos(heading);
        double lineEndY = robotYInches + lineLength * Math.sin(heading);
        fieldOverlay.setStroke("white");
        fieldOverlay.setStrokeWidth(2);
        fieldOverlay.strokeLine(robotXInches, robotYInches, lineEndX, lineEndY);

        // Draw desired movement vector (green line showing forward/strafe direction)
        double movementMagnitude = Math.sqrt(forward * forward + strafe * strafe);
        if (movementMagnitude > 0.01) {
            // Scale the movement vector for visibility (20 inches at full power)
            double vectorScale = 20.0;
            double movementAngle = Math.atan2(forward, strafe);
            double movementEndX = robotXInches + vectorScale * movementMagnitude * Math.cos(movementAngle);
            double movementEndY = robotYInches + vectorScale * movementMagnitude * Math.sin(movementAngle);

            fieldOverlay.setStroke("green");
            fieldOverlay.setStrokeWidth(2);
            fieldOverlay.strokeLine(robotXInches, robotYInches, movementEndX, movementEndY);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Telemetry
        telemetry.addData("=== DIAGNOSTICS ===", "");
        telemetry.addData("Inference Success?", inferenceSuccess);
        telemetry.addData("Inference Time (ms)", inferenceTime);
        telemetry.addData("Obs Checksum", "%.4f", obsChecksum);
        telemetry.addData("Action Checksum", "%.4f", actionChecksum);

        telemetry.addData("=== Position ===", "");
        telemetry.addData("Position", "%.2f, %.2f", robotX, robotY);
        telemetry.addData("Target", "%.2f, %.2f", targetX, targetY);
        telemetry.addData("Rotation", "%.2f deg", currentPose.getHeading(AngleUnit.DEGREES));

        telemetry.addData("=== Raw Observations ===", "");
        for (int i = 0; i < observations.length; i++) {
            telemetry.addData("obs[" + i + "]", "%.3f", observations[i]);
        }

        telemetry.addData("=== Actions ===", "");
        telemetry.addData("Actions", "F:%.3f S:%.3f R:%.3f", forward, strafe, rotation);

        telemetry.update();
    }
}
