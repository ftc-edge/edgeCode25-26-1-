package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.components.Constants.initHeading;
import static org.firstinspires.ftc.teamcode.components.Constants.startX;
import static org.firstinspires.ftc.teamcode.components.Constants.startY;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.auto.YAIBA;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;

@TeleOp
public class imuTest extends OpMode {
    public YAIBA auto;
    private Constants constants;
    private GoBildaPinpointDriver odo;
    Drive drive;

    Pose2D currentPose;

    private float offset;

    private float desiredHeading = 90f;

    private float currentHeading;
    @Override
    public void init() {
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(16.5, -19, DistanceUnit.CM);
        Pose2D startPose = new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.DEGREES, initHeading);
        odo.setPosition(startPose);
    }

    @Override
    public void loop() {
        odo.update();
        currentPose = odo.getPosition();

        // Convert to positive 0-360 degrees
        double rawDeg = Math.toDegrees(currentPose.getHeading(AngleUnit.RADIANS));
        double positiveHeading = (rawDeg % 360 + 360) % 360;

        Drive.setPower(0, 0, imuCorrection());

        telemetry.addData("Positive Heading", positiveHeading);
        telemetry.update();
    }

    private float imuCorrection() {
        float rawHeading = (float) Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES));
        currentHeading = (rawHeading % 360 + 360) % 360;
        // Ensure your desiredHeading is also in the 0-360 range (e.g., 90.0f)
        offset = (float) (desiredHeading - currentHeading);

        // IMPORTANT: Even with 0-360 values, you must still normalize the OFFSET
        // so the robot doesn't turn 350 degrees to reach a 10-degree target.
        while (offset > 180) offset -= 360;
        while (offset <= -180) offset += 360;

        if (Math.abs(offset) < 0.5f) return 0;
        return (offset * constants.imuKp);
    }
}
