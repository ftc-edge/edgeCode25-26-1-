package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.trapezoidalPIDSpindexer;

@TeleOp
public class spindexPIDTest extends OpMode {

    private trapezoidalPIDSpindexer spindex;
    public static float currentPosition;
    public static float targetPosition;
    public static float power;
    private FtcDashboard dashboard;

    Gamepad prevGamepad1 = new Gamepad();
    @Override
    public void init() {
        spindex = new trapezoidalPIDSpindexer(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        if(gamepad1.cross && !prevGamepad1.cross){
            spindex.startMove(spindex.spinMotor.getCurrentPosition(), 1);
        }
        if(gamepad1.triangle && !prevGamepad1.triangle){
            spindex.startMove(spindex.spinMotor.getCurrentPosition(), -1);
        }
        if(gamepad1.circle && !prevGamepad1.circle){
            spindex.startMove(spindex.spinMotor.getCurrentPosition(), 2);
        }
        if(gamepad1.square && !prevGamepad1.square){
            spindex.startMove(spindex.spinMotor.getCurrentPosition(), -2);
        }


        power = (float) spindex.update(spindex.spinMotor.getCurrentPosition());

        spindex.spinMotor.setPower(power);

        prevGamepad1.copy(gamepad1);

        targetPosition = spindex.targetPosition;
        currentPosition = spindex.spinMotor.getCurrentPosition();


        telemetry.addData("currentPosition", currentPosition);
        telemetry.addData("targetPosition", targetPosition);
        telemetry.addData("power", power);
        telemetry.update();

    }
}
