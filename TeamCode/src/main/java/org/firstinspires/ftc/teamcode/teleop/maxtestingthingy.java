package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.Hood;

@TeleOp
public class maxtestingthingy extends OpMode {

//    Intake intake;
    Spindex spindex;
    Turret turret;

    Hood hood;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    private float servoPosition;

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

        if(gamepad1.a && !prevGamepad1.a) {
            turret.togglePower(0.7f);
        }

        //servo.setPosition(servoPosition);

        if(gamepad1.dpad_right && !prevGamepad1.dpad_right){
            spindex.spinUp();
        }
        if(gamepad1.dpad_left && !prevGamepad1.dpad_left){
            spindex.spinTurns(-1);
        }

        if(gamepad1.dpad_up){
            hood.incrementPosition(0.002);
        }
        if(gamepad1.dpad_down){
            hood.incrementPosition(-0.002);
        }

        telemetry.addData("Spindex position", spindex.getCurrentPosition());
        telemetry.addData("Hood Position", hood.getPosition());

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
    }
}
