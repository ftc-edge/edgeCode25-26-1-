package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretSpin;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class teleop extends OpMode {
    //    Intake intake;
    Drive drive;
    Spindex spindex;
    Turret turret;

    TurretSpin turretSpin;

    Hood hood;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
//        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        hood = new Hood(hardwareMap);

    }

    @Override
    public void loop() {
//        intake.setPower(1);

        if (gamepad1.a && !prevGamepad1.a) {
            turret.togglePower(0.7f);
        }

        //servo.setPosition(servoPosition);

        if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
            spindex.spinUp();
        }
        if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
            spindex.spinTurns(-1);
        }

        if (gamepad1.dpad_up) {
            hood.incrementPosition(0.002);
        }
        if (gamepad1.dpad_down) {
            hood.incrementPosition(-0.002);
        }

        telemetry.addData("Spindex position", spindex.getCurrentPosition());
        telemetry.addData("Hood Position", hood.getPosition());

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
    }
}
