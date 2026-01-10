package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.components.Constants.initHeading;
import static org.firstinspires.ftc.teamcode.components.Constants.startX;
import static org.firstinspires.ftc.teamcode.components.Constants.startY;

import com.acmerobotics.dashboard.config.Config;
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
@Config
public class imuTest extends OpMode {
    public YAIBA auto;

    private GoBildaPinpointDriver odo;
    Drive drive;

    Pose2D currentPose;

    private float offset;

    private float desiredHeading = Constants.desiredHeading;

    private float currentHeading;
    @Override
    public void init() {
        drive = new Drive(hardwareMap);
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
        double rawDeg = Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES));
        double positiveHeading = (rawDeg % 360 + 360) % 360;

        Drive.setPower(0, 0, imuCorrection(positiveHeading));

        telemetry.addData("rad", currentPose.getHeading(AngleUnit.RADIANS));
        telemetry.addData("Positive Heading", positiveHeading);
        telemetry.addData("Desired Heading", desiredHeading);
        telemetry.addData("Correction", imuCorrection(positiveHeading));
        telemetry.update();
    }

    private float imuCorrection(double currentHeading) {
        // Ensure your desiredHeading is also in the 0-360 range (e.g., 90.0f)
        offset = -1 * ((float) AngleUnit.normalizeDegrees(desiredHeading - currentHeading));

//        // IMPORTANT: Even with 0-360 values, you must still normalize the OFFSET
//        // so the robot doesn't turn 350 degrees to reach a 10-degree target.
//        while (offset > 180) offset -= 360;
//        while (offset <= -180) offset += 360;

        telemetry.addData("offset", offset);
        if (Math.abs(offset) < 1f) return 0;
        return Math.copySign(Math.abs(offset) < 20f ? Constants.imuKp / 2 : Constants.imuKp, offset);
    }
}
