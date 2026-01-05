package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class Spindex {
    int currentPosition = 0;

    public static int spindexRotation = 538;
    public static int spinUpNumRotations = 2;
    public static float spinPower = 0.2f;
    public static float spinUpPower = 0.8f;


    public enum targetMotif{
        GPP,
        PPG,
        PGP;
    }

    DcMotor spinMotor;
    public Spindex(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(0);
        spinMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void spinTurns(int numTurns){
        // if a pressed
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition() + (numTurns * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinPower);
    }

    public void spinUp(){
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition() - (spinUpNumRotations * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinUpPower);
    }

    public void setPower(double power){
        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinMotor.setPower(power);
    }

    public void stop(){
        spinMotor.setPower(0);
    }
    public int getCurrentPosition(){
        return spinMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        return spinMotor.getTargetPosition();
    }
}
