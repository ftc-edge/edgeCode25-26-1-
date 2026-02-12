package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.SpindexPID;
import org.firstinspires.ftc.teamcode.components.trapezoidalPIDSpindexer;

@TeleOp
public class spindexPIDTest extends OpMode {

    public static float currentPosition;
    public static float targetPosition;

    Gamepad prevGamepad1 = new Gamepad();

    SpindexPID pid;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pid = new SpindexPID(hardwareMap);
    }

    @Override
    public void loop() {


        if(gamepad1.cross && !prevGamepad1.cross){
            pid.setTargetStep(1);
        }
        if(gamepad1.triangle && !prevGamepad1.triangle){
            pid.setTargetStep(-1);
        }
        if(gamepad1.circle && !prevGamepad1.circle){
            pid.setTargetStep(2);
        }
        if(gamepad1.square && !prevGamepad1.square){
            pid.setTargetStep(-2);
        }
        if(gamepad1.left_bumper && !prevGamepad1.left_bumper){
            pid.setTargetStep(-3);
        }
        if(gamepad1.right_bumper && !prevGamepad1.right_bumper){
            pid.setTargetStep(3);
        }

        pid.update();


//        power = (float) spindex.update(spindexer.spinMotor.getCurrentPosition());
//
//        spindexer.spinMotor.setPower(power);

        prevGamepad1.copy(gamepad1);

        targetPosition = (float) pid.getTargetPosition();
        currentPosition = pid.getCurrentPosition();


        telemetry.addData("currentPosition", currentPosition);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("power", pid.isAtTarget());
        telemetry.update();

    }
}
