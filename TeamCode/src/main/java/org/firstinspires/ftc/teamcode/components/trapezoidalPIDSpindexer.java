package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;

@Config
public class trapezoidalPIDSpindexer {
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

    public double integralSum = 0;
    public double lastError = 0;
    public static double deadband = 5;
    public int targetPosition = 0;

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
    public trapezoidalPIDSpindexer(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotorEx.class, "spindex");
        spinMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        spinMotor.setTargetPosition(0);
        //spinMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spinMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spinMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        spinMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

        // Constants - Tune these for your specific motor/spindexer
        private double TICKS_PER_DEGREE = 1.068; // Example value
        private double MAX_V = 2787.625; // ticks per second
        private double MAX_A = 1107.36; // ticks per second

        // State Variables
        public static double startPos, relativeTarget, finalTarget;
        private double accelTime, cruiseTime, totalTime;
        private double accelDist, cruiseDist;
        private double startTime = -1;
        private boolean isRunning = false;

        // Call this once to start a 120-degree move
        public void startMove(double currentMotorPos, double numTurns) {
            startPos = currentMotorPos;
            //relativeTarget = degrees * TICKS_PER_DEGREE;
            relativeTarget = (spindexRotation / 3) * numTurns;
            finalTarget = startPos + relativeTarget;

            // 1. Calculate the theoretical max velocity reach
            // Equation: v^2 = 2 * a * d
            double theoreticalMaxV = Math.sqrt(relativeTarget * MAX_A);
            double activeMaxV = Math.min(MAX_V, theoreticalMaxV);

            // 2. Calculate Timing
            accelTime = activeMaxV / MAX_A;
            accelDist = 0.5 * MAX_A * Math.pow(accelTime, 2);

            cruiseDist = relativeTarget - (2 * accelDist);
            if (cruiseDist < 0) cruiseDist = 0; // Handle Triangle Profile
            cruiseTime = cruiseDist / activeMaxV;

            totalTime = (2 * accelTime) + cruiseTime;
            startTime = System.currentTimeMillis() / 1000.0;
            isRunning = true;
        }

        // Call this every single frame in your main loop
        public double update(double currentPos) {
            if (!isRunning) return 0;

            double elapsedTime = (System.currentTimeMillis() / 1000.0) - startTime;
            double profilePos = 0;

            // Phase 1: Acceleration
            if (elapsedTime < accelTime) {
                profilePos = 0.5 * MAX_A * Math.pow(elapsedTime, 2);
            }
            // Phase 2: Cruise
            else if (elapsedTime < accelTime + cruiseTime) {
                double cruiseElapsed = elapsedTime - accelTime;
                profilePos = accelDist + (Math.min(MAX_V, Math.sqrt(relativeTarget * MAX_A)) * cruiseElapsed);
            }
            // Phase 3: Deceleration
            else if (elapsedTime < totalTime) {
                double decelElapsed = elapsedTime - accelTime - cruiseTime;
                double vAtDecel = Math.min(MAX_V, Math.sqrt(relativeTarget * MAX_A));
                profilePos = accelDist + cruiseDist + (vAtDecel * decelElapsed) - (0.5 * MAX_A * Math.pow(decelElapsed, 2));
            }
            // Move Complete
            else {
                profilePos = relativeTarget;
                if (Math.abs(currentPos - finalTarget) < 10) isRunning = false;
            }

            // Calculate PID output based on the "Moving Target"
            double instantTarget = startPos + profilePos;
            double error = instantTarget - currentPos;

            // Simplified PID (No integral for clarity, add if needed)
            double power = (Kp * error);

            return power;
        }
    }
