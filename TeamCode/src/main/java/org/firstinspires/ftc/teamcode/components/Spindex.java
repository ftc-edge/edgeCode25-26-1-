package org.firstinspires.ftc.teamcode.components;

import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
public class Spindex {
    public static int spinUpNumRotations = 1;

    public static int beforeShootAdjust = 60;
    public static int adjustDelayMs = 700;
    public static int adjustDelay2Ms = 450;
    public static int readColorDelayMs = 650;
    public static int busyTimerMs = 150;

    public boolean ifBusyTimer = false;
    private boolean isBusy = false;

    private SpindexPID pid;

    private static ElapsedTime shootTimer = new ElapsedTime();
    private static ElapsedTime busyTimer = new ElapsedTime();
    public int shot = 0;
    public boolean shooting = false;

    public  double power = 0;

    //pid variables

    private double targetPosition = 0; // In encoder ticks

    // Tuning coefficients (Need to be tuned for your specific motor/weight)
    public static double kP = 0.005, kI = 0, kD = 0.0001;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    public DcMotor spinMotor;
    SpindexPID profile;
    public static float t_a;
    public static float t_c;
    public static float t_total;

    public static float d_a;
    public static float d_c;

    public static float maxAccel;
    public static float maxVel;
    public Spindex(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(0);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);

        pid = new SpindexPID(hardwareMap);

        //profile = new SpindexPID();
    }

    public void update(){
        //isBusy = pid.isRunning;
//        power = pid.update(spinMotor.getCurrentPosition());
//        spinMotor.setPower(power);
       pid.update();

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
            pid.spin(beforeShootAdjust);
            shootTimer.reset(); shot++; return;
        }
        if (shot == 1 && shootTimer.milliseconds() >= adjustDelayMs) {
            pid.spin(-beforeShootAdjust);
            shootTimer.reset(); shot++; return;
        }
        if(shot >= 2 && !spinMotor.isBusy() && shootTimer.milliseconds() >= adjustDelay2Ms){
            if(shot <= 4){ // 2, 3, 4, we dont check
                spinUp();
                shootTimer.reset();
                shot++;
            }
            else if(spindexColor.getColor() == "PURPLE" || spindexColor.getColor() == "GREEN"){
                spinUp();
                shootTimer.reset();
                shot++;
            }else{
                shot = -4;
                shootTimer.reset();
            }
        }
        if(shot < -1 && shootTimer.milliseconds() >= readColorDelayMs){ // adjust so that the camera reads all ball positions, shot = -4, -3, -2
            shot++;
            spinTurns(1);
            shootTimer.reset();
        }
        if(shot == -1) {
            shooting = false;
            shot = 0;
            shootTimer.reset();
        }
    }

    public void stop(){
        spinMotor.setPower(0);
        pid.stop();
    }


    //    public void updateTimer(){
//        if (abs(getCurrentPosition() - getTargetPosition()) > 18) {
//            isBusy = true;
//            ifBusyTimer = false;
//            return;
//        }
//
//        if(ifBusyTimer){
//            if(busyTimer.milliseconds() > busyTimerMs){
//                ifBusyTimer = false;
//                isBusy = false;
//            }
//            return;
//        }
//
//        ifBusyTimer = true;
//        busyTimer.reset();
//    }

    public void spinTurns(int numTurns){
        pid.setTargetStep(numTurns);
    }

    public void spinUp(){
        pid.setTargetStep(-1 * spinUpNumRotations);
    }




}
