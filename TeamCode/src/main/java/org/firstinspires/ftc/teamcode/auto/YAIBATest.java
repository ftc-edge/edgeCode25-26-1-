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

    // Hardware
    private GoBildaPinpointDriver odo;

    // Odometry (you'll need to implement this based on your setup)
    private double robotX = 0, robotY = 0;
    private double targetX = 60.0, targetY = 0.0; // meters

    Drive drive;

    public Pose2D currentPose;

    public float targetAngle = 0f;
    public float stageX;
    public float stageY;

    /**
     * Build observation array matching Unity's CollectObservations
     */
    private float[] buildObservations() {
        float[] obs = new float[9];

        updateOdometry();
        // Get current heading in radians
        float currentHeading = (float) currentPose.getHeading(AngleUnit.RADIANS);
        robotX = currentPose.getX(DistanceUnit.CM) / 10f;
        robotY = currentPose.getY(DistanceUnit.CM) / 10f;
        // 1-2: Relative position to target
        obs[0] = (float)(targetX - robotX);
        obs[1] = (float)(targetY - robotY);

        // 3-4: Current heading (sin, cos)
        obs[2] = (float)Math.sin(currentHeading);
        obs[3] = (float)Math.cos(currentHeading);

        // 5-6: Desired rotation to face target
        obs[4] = (float)Math.sin(targetAngle);
        obs[5] = (float)Math.cos(targetAngle);

        // 7: Stage active (for multi-stage paths)
        obs[6] = 0.0f; // Set to 1.0f if using staged waypoints

        if(obs[6] == 1){
            obs[7] = stageX;
            obs[8] = stageY;
        }else{
            obs[7] = 0f;
            obs[8] = 0f;
        }

        return obs;
    }

    /**
     * Update robot position using odometry
     * Implement based on your wheel encoders/IMU setup
     */
    private void updateOdometry() {
        odo.update();
        currentPose = odo.getPosition();
    }

    @Override
    public void init() {
            drive = new Drive(hardwareMap);
            odo = hardwareMap.get(GoBildaPinpointDriver .class, "odo");
            odo.resetPosAndIMU();
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setOffsets(12, -17.5, DistanceUnit.CM);
            Pose2D startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.RADIANS, -1.578);
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
        // Update odometry (implement based on your sensors)

        // Build observations (MUST match Unity order exactly)
        float[] observations = buildObservations();

        // Get AI predictions
        float[] actions;

        try {
            actions = model.predict(observations);
        } catch (Exception e) {
            telemetry.addData("Error", "Inference failed: " + e.getMessage());
            telemetry.update();
            actions = new float[]{0, 0, 0};
        }

        // Extract actions (order from Unity: strafe, forward, rotation)
        float strafe = actions[0];
        float forward = actions[1];
        float rotation = actions[2];

        // Apply to mecanum drive
//        drive.setPower(forward, strafe, rotation);

        // Telemetry
        telemetry.addData("Position", "%.2f, %.2f", robotX, robotY);
        telemetry.addData("Target", "%.2f, %.2f", targetX, targetY);
        telemetry.addData("Rotation", "%.2f, %.2f", currentPose.getHeading(AngleUnit.RADIANS), targetAngle);
        telemetry.addData("Actions", "F:%.2f S:%.2f R:%.2f", forward, strafe, rotation);
        telemetry.update();
    }
}
