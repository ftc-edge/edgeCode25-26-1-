package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.abs;
import static java.lang.Runtime.getRuntime;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Spindex {
    int currentPosition = 0;

    public static float spindexRotation = 384.5f; // originally 538
    public static int spinUpNumRotations = 1;
    public static float spinPower = 1f;

    public static float spinUpPower = 1f;
    public static float adjustPower = 0.35f;

    public static int beforeShootAdjust = 70;


    public static int shootDelayMs = 800;
    public static int adjustDelayMs = 700;
    public static int adjustDelay2Ms = 350;
    public static int busyTimerMs = 150;

    public static float Kp = 0.005f;
    public static float Ki = 0.001f;
    public static float Kd = 0.01f;

    public double integralSum = 0;
    public double lastError = 0;
    public static double deadband = 5;
    public int targetPosition;

    public boolean ifBusyTimer = false;
    public boolean isBusy = false;

    private static ElapsedTime shootTimer = new ElapsedTime();
    private static ElapsedTime busyTimer = new ElapsedTime();
    public int shot = 0;
    public boolean shooting = false;

    Color spindexColor;
    public enum targetMotif{
        GPP,
        PPG,
        PGP;
    }

    public DcMotor spinMotor;
    public Spindex(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(0);
        //spinMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);

        //spindexColor = new Color(hardwareMap);
    }

    public void spinTurns(int numTurns){
        // if a pressed
//        spinMotor.setTargetPosition(spinMotor.getTargetPosition() + (int) (numTurns * spindexRotation / 3));
//        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        spinMotor.setPower(spinPower);
    }

//    public void setSpin(int numTurns){
//        double timer = getRuntime(); // Start a clock
//
//        while (elapsedTime < total_time) {
//            double elapsedTime = getRuntime() - timer;
//            double instantTarget = 0;
//
//            if (elapsedTime < accel_time) {
//                // Phase 1: Acceleration
//                instantTarget = 0.5 * max_a * Math.pow(elapsedTime, 2);
//            }
//            else if (elapsedTime < accel_time + cruise_time) {
//                // Phase 2: Cruise
//                double cruiseElapsed = elapsedTime - accel_time;
//                instantTarget = accel_dist + (max_v * cruiseElapsed);
//            }
//            else {
//                // Phase 3: Deceleration
//                double decelElapsed = elapsedTime - accel_time - cruise_time;
//                instantTarget = accel_dist + cruise_dist + (max_v * decelElapsed) - (0.5 * max_a * Math.pow(decelElapsed, 2));
//            }
//        }
//
//    }

    public double calculatePID(float targetPosition, float currentPosition){
        double error = targetPosition - currentPosition;
        double P = Kp * error;
        if(Math.abs(error) < 50){
            integralSum += error;
        }
        double I = Ki * integralSum;

        double D = Kd * (error - lastError);

        double power = P + I + D;

        if(Math.abs(error) < deadband){
            spinMotor.setPower(0);
            return 0;
        }

        return power;
    }

    public void spinUp(){
        spinMotor.setTargetPosition(spinMotor.getTargetPosition() - (int) (spinUpNumRotations * spindexRotation / 3));
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
            ifBusyTimer = false;
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
        return !isBusy;
    }

    public void startShootConsecutive(){
        shooting = true;
        shootTimer.reset();
    }
    public void shootConsecutive(Color spindexColor){
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
            spinMotor.setPower(adjustPower);
            shootTimer.reset(); shot++; return;
        }
        if(shot >= 2 && !spinMotor.isBusy() && shootTimer.milliseconds() >= adjustDelay2Ms){
            if(spindexColor.getColor() == "PURPLE" || spindexColor.getColor() == "GREEN"){
                spinUp();
                shootTimer.reset();
                shot++;
            }else{
                shooting = false;
                shot = 0;
                shootTimer.reset();
            }
        }
//        if (shot == 2 && shootTimer.milliseconds() >= adjustDelay2Ms) {
//            spinUp();
//            shootTimer.reset(); shot++; return;
//        }
//        if((shot == 3 || shot == 4) && shootTimer.milliseconds() >= shootDelayMs){
//            spinUp();
//            shootTimer.reset(); shot++; return;
//        }
//        if (shot >= 5){
//            shooting = false; shot = 0;
//            shootTimer.reset();
//        }
    }

    public void stop(){
        spinMotor.setPower(0);
    }
    public int getCurrentPosition(){
        return spinMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        //return spinMotor.getTargetPosition();
        return targetPosition;
    }
}
