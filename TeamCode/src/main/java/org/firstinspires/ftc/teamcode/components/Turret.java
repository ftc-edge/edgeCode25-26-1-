package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;


@Config
public class Turret {

    private DcMotorEx TurretX1;
    private DcMotorEx TurretX2;

    public static double kP = -0.02;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;
    public static double tolerance = 3.0; // RPM tolerance

    // Adjust this based on your motors (e.g., 537.7 for goBILDA 5202/5203)
    public static double TICKS_PER_REV = 537.7;

    public static float targetRPM1 = 40f;

    private PIDFController pidf;

    public static int disableShoot1 = 0;
    public static int disableShoot2 = 0;

    private double targetRPM = 0;
    private long lastTime;

    public Turret(HardwareMap hardwareMap) {
        pidf = new PIDFController(kP, kI, kD, kF);
        pidf.setTolerance(tolerance);

        if (disableShoot1 == 0) {
            TurretX1 = hardwareMap.get(DcMotorEx.class, "motor1");
            TurretX1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            TurretX1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TurretX1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            TurretX1.setDirection(DcMotorEx.Direction.FORWARD);
        }

        if (disableShoot2 == 0) {
            TurretX2 = hardwareMap.get(DcMotorEx.class, "motor2");
            TurretX2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            TurretX2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            TurretX2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            TurretX2.setDirection(DcMotorEx.Direction.REVERSE);
        }

        lastTime = System.nanoTime();
    }

    public void loop() {
        // Update PID coefficients from dashboard if changed
        pidf.setPIDF(kP, kI, kD, kF);
        pidf.setTolerance(tolerance);

        // Get current RPM from motor velocity
        double currentRPM = 0;
        if (disableShoot1 == 0 && TurretX1 != null) {
            double ticksPerSecond = TurretX1.getVelocity();
            currentRPM = (ticksPerSecond * 60.0) / TICKS_PER_REV;
        }

        pidf.setSetPoint(targetRPM);

        // Calculate PID output
        double output = pidf.calculate(currentRPM, targetRPM);

        // Apply power to motors
        if (disableShoot1 == 0 && TurretX1 != null) {
            TurretX1.setPower(output);
        }
        if (disableShoot2 == 0 && TurretX2 != null) {
            TurretX2.setPower(output);
        }
    }

    public void setTargetRPM(double target) {
        targetRPM = target;
    }

    public void togglePower(double target) {
        if (targetRPM != 0) {
            targetRPM = 0;
        } else {
            targetRPM = target;
        }
    }

    public void stop() {
        targetRPM = 0;
        if (disableShoot1 == 0 && TurretX1 != null) {
            TurretX1.setPower(0);
        }
        if (disableShoot2 == 0 && TurretX2 != null) {
            TurretX2.setPower(0);
        }
    }

    public double getCurrentRPM() {
        if (disableShoot1 == 0 && TurretX1 != null) {
            double ticksPerSecond = TurretX1.getVelocity();
            return (ticksPerSecond * 60.0) / TICKS_PER_REV;
        }
        return 0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean getAtPoint(){
        return pidf.atSetPoint();
    }

    public boolean atTarget() {
        return pidf.atSetPoint();
    }
    public double getPower() {
        if (disableShoot1 == 0 && TurretX1 != null) {
            return TurretX1.getPower();
        }
        return 0;
    }
}