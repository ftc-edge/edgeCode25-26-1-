package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Drive;

@Autonomous(name = "Move Goal Auto", group = "Simple")
@Config
public class moveGoalAuto extends LinearOpMode {

    // Tune these two numbers on your robot
    public static double FORWARD_POWER = 0.50; // small power for accuracy
    public static double STRAFE_POWER = 0.50; // small power for accuracy
    public static long MOVE1_MS = 700;         // start here, then adjust until it's ~2cm
    public static long MOVE2_MS = 1100;

    @Override
    public void runOpMode() {
        // Make sure Drive maps motors in its constructor/init
        Drive drive = new Drive(hardwareMap);

        telemetry.addLine("Ready. ");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Strafe left: negative strafe (matches your teleop: strafe = left_stick_x)
        drive.setPower(FORWARD_POWER, 0, 0.0);
        sleep(MOVE1_MS);

        drive.setPower(0, STRAFE_POWER, 0.0);
        sleep(MOVE2_MS);

        // Stop
        drive.setPower(0.0, 0.0, 0.0);

        // Small settle
        sleep(100);
    }
}
