package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class Outtake {
    MultipleTelemetry telemetry;

    private static float powerParamA = .12f;
    private static float powerParamB = .07f;
    private static float powerParamC = .32f;

    private static float defaultPower = .8f;

    public Outtake(HardwareMap hardwareMap, MultipleTelemetry telemetry){
        this.telemetry = telemetry;
    }

    private float calcPower(float distanceCM){
        // piecewise function for power
        // for distance <= 2m, use power = a(dist-b)^2+c
        float distance = distanceCM/100;
        float power = defaultPower;

        if (distance <= 2){
            power = (float)(powerParamA*Math.pow(distance-powerParamB,2)+powerParamC);
        }

        if (distance <= .6){
            // give up
            power = -1;
        }

        telemetry.addData("Calculated Outtake Power", power);

        return power;
    }
    public void shoot(float distanceCM){

    }
}
