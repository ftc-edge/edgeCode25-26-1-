package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Drive;

@Config
public class moveShootAuto extends LinearOpMode {
    // Tune these two numbers on your robot
    public static double STRAFE_POWER = 0.50; // small power for accuracy
    public static long MOVE_MS = 700;         // start here, then adjust until it's ~2cm

    @Override
    public void runOpMode() {
        // Make sure Drive maps motors in its constructor/init
        Drive drive = new Drive(hardwareMap);

        telemetry.addLine("Ready. Will strafe left ~2cm on start.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Strafe left: negative strafe (matches your teleop: strafe = left_stick_x)
        Drive.setPower(0.0, -STRAFE_POWER, 0.0);
        sleep(MOVE_MS);

        // Stop
        Drive.setPower(0.0, 0.0, 0.0);

        // Small settle
        sleep(100);
    }
}
