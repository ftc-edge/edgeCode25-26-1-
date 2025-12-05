package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Spin;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class teleop extends OpMode{
    Intake intake;

    Drive drive;

    Spin spin;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if(gamepad1.a & !prevGamepad1.a) {
            // smth
        }

        if(gamepad1.right_bumper) {
            intake.doIntake();
            spin.setPower(1);
        }
        if(gamepad1.left_bumper) {
            intake.doIntake();
            spin.setPower(-1);
        }

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);

        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        Drive.setPower(forward, strafe, pivot);


    }
}
