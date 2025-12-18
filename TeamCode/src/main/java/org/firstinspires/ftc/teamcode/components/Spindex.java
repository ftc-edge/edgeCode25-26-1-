package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;


@Config
public class Spindex {
    int currentPosition = 0;

    public static int spindexRotation = 538;
    public static int spinUpNumRotations = 2;
    public static float spinPower = 0.6f;
    public static float spinUpPower = 1f;


    public enum targetMotif{
        GPP,
        PPG,
        PGP;
    }

    DcMotor spinMotor;
    public Spindex(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotor.class, "spindex");
        spinMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(0);
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void spinTurns(int numTurns){
        // if a pressed
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition() + (numTurns * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinPower);
    }

    public void spinUp(){
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition() + (spinUpNumRotations * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinUpPower);
    }

    public int getCurrentPosition(){
        return spinMotor.getCurrentPosition();
    }

}
