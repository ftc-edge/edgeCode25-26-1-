package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.components.Constants.initHeading;
import static org.firstinspires.ftc.teamcode.components.Constants.startX;
import static org.firstinspires.ftc.teamcode.components.Constants.startY;

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

@TeleOp
public class YAIBATest extends OpMode {

    private BODYONNX model;

    private GoBildaPinpointDriver odo;

    private double robotX = 0, robotY = 0;
    private double targetX = 60.0, targetY = 30.0; // cm

    Drive drive;

    public Pose2D currentPose;
    public Pose2D startPose;

    public float targetAngle = 0f;
    public float stageX;
    public float stageY;
    boolean notStarted;

    /**
     * Build observation array matching Unity's CollectObservations
     */
    private float[] buildObservations() {
        float[] obs = new float[9];


        // Get current heading in DEGREES
        float currentHeading = (float) currentPose.getHeading(AngleUnit.DEGREES);
        robotX = currentPose.getX(DistanceUnit.CM) / 10f;
        robotY = currentPose.getY(DistanceUnit.CM) / 10f;
        // obs1/2: relatige position of the
        obs[0] = (float)(targetX - robotX);
        obs[1] = (float)(targetY - robotY);

        // obs3/4: sin and cos of the current angle
        obs[2] = (float)Math.sin(currentHeading);
        obs[3] = (float)Math.cos(currentHeading);

        // obs5/6: sin and cos of the desired angle
        obs[4] = (float)Math.sin(targetAngle);
        obs[5] = (float)Math.cos(targetAngle);

        //Obs 7: stage active, 1.0 if theres a intermediary stage
        obs[6] = 0.0f;

        if(obs[6] == 1){
            //obs8/9: intermediary stage location, 0 if nonexistent
            obs[7] = stageX;
            obs[8] = stageY;
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
        float[] actions;
        boolean inferenceSuccess = false;
        long inferenceTime = 0;

        try {
            long startTime = System.nanoTime();
            actions = model.predict(observations);
            inferenceTime = (System.nanoTime() - startTime) / 1_000_000;
            inferenceSuccess = true;
        } catch (Exception e) {
            telemetry.addData("Error", "Inference failed: " + e.getMessage());
            e.printStackTrace(); // CHECK LOGCAT FOR THIS
            actions = new float[]{0, 0, 0};
        }

        // Calculate action checksum
        float actionChecksum = actions[0] + actions[1] + actions[2];

        float strafe = actions[0];
        float forward = actions[1];
        float rotation = actions[2];

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
