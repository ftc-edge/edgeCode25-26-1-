package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Spindex {
    int currentPosition = 0;

    public static int spindexRotation = 538; // originally 538
    public static int spinUpNumRotations = 1;
    public static float spinPower = 0.35f;
    public static float spinUpPower = 0.8f;
    public static float adjustPower = 0.35f;

    public static int beforeShootAdjust = 88;


    public static int shootDelayMs = 800;
    public static int adjustDelayMs = 700;
    public static int adjustDelay2Ms = 200;
    public static int busyTimerMs = 150;
    public boolean ifBusyTimer = false;
    public boolean isBusy = false;

    public static int withinTargetDelayMs = 400;

    private static ElapsedTime shootTimer = new ElapsedTime();
    private static ElapsedTime busyTimer = new ElapsedTime();
    public int shot = 0;
    public boolean shooting = false;

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
        spinMotor.setTargetPosition(spinMotor.getTargetPosition() + (numTurns * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinPower);
    }

    public void spinUp(){
        spinMotor.setTargetPosition(spinMotor.getTargetPosition() - (spinUpNumRotations * spindexRotation / 3));
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setPower(spinUpPower);
    }

    public void setPower(double power){
        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spinMotor.setPower(power);
    }

    public void updateTimer(){
        if (abs(getCurrentPosition() - getTargetPosition()) > 18) {
            isBusy = true;
            return;
        }

        if(ifBusyTimer){
            if(busyTimer.milliseconds() > busyTimerMs){
                ifBusyTimer = false;
                isBusy = false;
            }
            return;
        }

        ifBusyTimer = true;
        busyTimer.reset();
    }
    public boolean withinTarget() {
        return isBusy;
    }

    public void startShootConsecutive(){
        shooting = true;
        shootTimer.reset();
    }
    public void shootConsecutive(){
        if (!shooting){
            return;
        }

        // 0th stage, turn back a little
        // first stage, turn forward a little
        // 2-4 stage, shoot each ball
        if (shot == 0) {
            spinMotor.setTargetPosition(spinMotor.getTargetPosition() + beforeShootAdjust);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(adjustPower);
            shootTimer.reset(); shot++; return;
        }
        if (shot == 1 && shootTimer.milliseconds() >= adjustDelayMs) {
            spinMotor.setTargetPosition(spinMotor.getTargetPosition() - beforeShootAdjust);
            spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spinMotor.setPower(spinPower);
            shootTimer.reset(); shot++; return;
        }
        if (shot == 2 && shootTimer.milliseconds() >= adjustDelay2Ms) {
            spinUp();
            shootTimer.reset(); shot++; return;
        }
        if((shot == 3 || shot == 4) && shootTimer.milliseconds() >= shootDelayMs){
            spinUp();
            shootTimer.reset(); shot++; return;
        }
        if (shot >= 5){
            shooting = false; shot = 0;
            shootTimer.reset();
        }
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
