//MIT License
//
//        Copyright (c) 2025 The Robotics Catalyst Foundation
//
//        Permission is hereby granted, free of charge, to any person obtaining a copy
//        of this software and associated documentation files (the "Software"), to deal
//        in the Software without restriction, including without limitation the rights
//        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//        copies of the Software, and to permit persons to whom the Software is
//        furnished to do so, subject to the following conditions:
//
//        The above copyright notice and this permission notice shall be included in all
//        copies or substantial portions of the Software.
//
//        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//        IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//        FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//        AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//        LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//        OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//        SOFTWARE.


package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RTPAxon {
    // Encoder for servo position feedback
    private final AnalogInput servoEncoder;
    // Continuous rotation servo
    private final CRServo servo;
    // Run-to-position mode flag
    private boolean rtp;
    // Current power applied to servo
    private double power;
    // Maximum allowed power
    private double maxPower;
    // Direction of servo movement
    private Direction direction;
    // Last measured angle
    private double previousAngle;
    // Accumulated rotation in degrees
    private double totalRotation;
    // Target rotation in degrees
    private double targetRotation;

    // PID controller coefficients and state
    private double kP;
    private double kI;
    private double kD;
    private double integralSum;
    private double lastError;
    private double maxIntegralSum; 
    private ElapsedTime pidTimer;

    // Initialization and debug fields
    public double STARTPOS;
    public int ntry = 0;
    public int cliffs = 0;
    public double homeAngle;

    // Direction enum for servo
    public enum Direction {
        FORWARD,
        REVERSE
    }

    // region constructors

    // Basic constructor, defaults to FORWARD direction
    public RTPAxon(CRServo servo, AnalogInput encoder) {
        rtp = true;
        this.servo = servo;
        servoEncoder = encoder;
        direction = Direction.FORWARD;
        initialize();
    }

    // Constructor with explicit direction
    public RTPAxon(CRServo servo, AnalogInput encoder, Direction direction) {
        this(servo, encoder);
        this.direction = direction;
        initialize();
    }

    // Initialization logic for servo and encoder
    private void initialize() {
        servo.setPower(0);
        try {
            Thread.sleep(50);
        } catch (InterruptedException ignored) {
        }

        // Try to get a valid starting position
        do {
            STARTPOS = getCurrentAngle();
            if (Math.abs(STARTPOS) > 1) {
                previousAngle = getCurrentAngle();
            } else {
                try {
                    Thread.sleep(50);
                } catch (InterruptedException ignored) {
                }
            }
            ntry++;
        } while (Math.abs(previousAngle) < 0.2 && (ntry < 50));

        totalRotation = 0;
        homeAngle = previousAngle;

        // Default PID coefficients
        kP = 0.015; 
        kI = 0.0005; 
        kD = 0.0025;
        integralSum = 0.0;
        lastError = 0.0;
        maxIntegralSum = 100.0;
        pidTimer = new ElapsedTime();
        pidTimer.reset();

        maxPower = 0.25;
        cliffs = 0;
    }
    // endregion

    // Set servo direction
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    // Set power to servo, respecting direction and maxPower
    public void setPower(double power) {
        this.power = Math.max(-maxPower, Math.min(maxPower, power));
        servo.setPower(this.power * (direction == Direction.REVERSE ? -1 : 1));
    }

    // Get current power
    public double getPower() {
        return power;
    }

    // Set maximum allowed power
    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    // Get maximum allowed power
    public double getMaxPower() {
        return maxPower;
    }

    // Enable or disable run-to-position mode
    public void setRtp(boolean rtp) {
        this.rtp = rtp;
        if (rtp) {
            resetPID();
        }
    }

    // Get run-to-position mode state
    public boolean getRtp() {
        return rtp;
    }

    // Set PID P coefficient
    public void setKP(double kP) {
        this.kP = kP;
    }

    // Set PID I coefficient and reset integral
    public void setKI(double kI) {
        this.kI = kI;
        resetIntegral(); 
    }

    // Set PID D coefficient
    public void setKD(double kD) {
        this.kD = kD;
    }

    // Set all PID coefficients
    public void setPidCoeffs(double kP, double kI, double kD){
        setKP(kP);
        setKI(kI);
        setKD(kD);
    }

    // Get PID P coefficient
    public double getKP() {
        return kP;
    }

    // Get PID I coefficient
    public double getKI() {
        return kI;
    }

    // Get PID D coefficient
    public double getKD() {
        return kD;
    }

    // Set only P coefficient (alias)
    public void setK(double k) {
        setKP(k);
    }

    // Get only P coefficient (alias)
    public double getK() {
        return getKP();
    }

    // Set maximum allowed integral sum
    public void setMaxIntegralSum(double maxIntegralSum) {
        this.maxIntegralSum = maxIntegralSum;
    }

    // Get maximum allowed integral sum
    public double getMaxIntegralSum() {
        return maxIntegralSum;
    }

    // Get total rotation since initialization
    public double getTotalRotation() {
        return totalRotation;
    }

    // Get current target rotation
    public double getTargetRotation() {
        return targetRotation;
    }

    // Increment target rotation by a value
    public void changeTargetRotation(double change) {
        targetRotation += change;
    }

    // Set target rotation and reset PID
    public void setTargetRotation(double target) {
        targetRotation = target;
        resetPID();
    }

    // Get current angle from encoder (in degrees)
    public double getCurrentAngle() {
        if (servoEncoder == null) return 0;
        return (servoEncoder.getVoltage() / 3.3) * (direction.equals(Direction.REVERSE) ? -360 : 360);
    }

    // Check if servo is at target (default tolerance)
    public boolean isAtTarget() {
        return isAtTarget(5);
    }

    // Check if servo is at target (custom tolerance)
    public boolean isAtTarget(double tolerance) {
        return Math.abs(targetRotation - totalRotation) < tolerance;
    }

    // Force reset total rotation and PID state
    public void forceResetTotalRotation() {
        totalRotation = 0;
        previousAngle = getCurrentAngle();
        resetPID();
    }

    // Reset PID controller state
    public void resetPID() {
        resetIntegral();
        lastError = 0;
        pidTimer.reset();
    }

    // Reset integral sum
    public void resetIntegral() {
        integralSum = 0;
    }

    // Main update loop: updates rotation, computes PID, applies power
    public synchronized void update() {
        double currentAngle = getCurrentAngle();
        double angleDifference = currentAngle - previousAngle;

        // Handle wraparound at 0/360 degrees
        if (angleDifference > 180) {
            angleDifference -= 360;
            cliffs--;
        } else if (angleDifference < -180) {
            angleDifference += 360;
            cliffs++;
        }

        // Update total rotation with wraparound correction
        totalRotation = currentAngle - homeAngle + cliffs * 360;
        previousAngle = currentAngle;

        if (!rtp) return;

        double dt = pidTimer.seconds();
        pidTimer.reset();

        // Ignore unreasonable dt values
        if (dt < 0.001 || dt > 1.0) {
            return;
        }

        double error = targetRotation - totalRotation;

        // PID integral calculation with clamping
        integralSum += error * dt;
        integralSum = Math.max(-maxIntegralSum, Math.min(maxIntegralSum, integralSum));

        // Integral wind-down in deadzone
        final double INTEGRAL_DEADZONE = 2.0;
        if (Math.abs(error) < INTEGRAL_DEADZONE) {
            integralSum *= 0.95;
        }

        // PID derivative calculation
        double derivative = (error - lastError) / dt;
        lastError = error;

        // PID output calculation
        double pTerm = kP * error;
        double iTerm = kI * integralSum;
        double dTerm = kD * derivative;

        double output = pTerm + iTerm + dTerm;

        // Deadzone for output
        final double DEADZONE = 0.5;
        if (Math.abs(error) > DEADZONE) {
            double power = Math.min(maxPower, Math.abs(output)) * Math.signum(output);
            setPower(power);
        } else {
            setPower(0);
        }
    }

    // Log current state for telemetry/debug
    @SuppressLint("DefaultLocale")
    public String log() {
        return String.format(
                "Current Volts: %.3f\n" +
                        "Current Angle: %.2f\n" +
                        "Total Rotation: %.2f\n" +
                        "Target Rotation: %.2f\n" +
                        "Current Power: %.3f\n" +
                        "PID Values: P=%.3f I=%.3f D=%.3f\n" +
                        "PID Terms: Error=%.2f Integral=%.2f",
                servoEncoder.getVoltage(),
                getCurrentAngle(),
                totalRotation,
                targetRotation,
                power,
                kP, kI, kD,
                targetRotation - totalRotation,
                integralSum
        );
    }

    // TeleOp test class for manual tuning and testing
    @TeleOp(name = "Cont. Rotation Axon Test", group = "test")
    public static class CRAxonTest extends LinearOpMode {

        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            CRServo left = hardwareMap.crservo.get("leftServo");
            CRServo right = hardwareMap.crservo.get("rightServo");
            AnalogInput encoder = hardwareMap.get(AnalogInput.class, "servoEncoder");
            //Gamepad gamepad1 = new Gamepad(gamepad1, gamepad2);
            RTPAxon leftServo = new RTPAxon(left, encoder);
            //RTPAxon rightServo = new RTPAxon(right, encoder, Direction.REVERSE);

            leftServo.initialize();
            //rightServo.initialize();

            leftServo.setRtp(true);
            //rightServo.setRtp(true);

            waitForStart();

            while (!isStopRequested()) {
                //gamepad1.copyStates();
                leftServo.update();
                //rightServo.update();

                right.setPower(left.getPower());

                // Manual controls for target and PID tuning
                if (gamepad1.dpad_up) {
                    leftServo.changeTargetRotation(1);
                    //rightServo.changeTargetRotation(15);
                }
                if (gamepad1.dpad_down) {
                    leftServo.changeTargetRotation(-1);
                    //rightServo.changeTargetRotation(-15);
                }
                if (gamepad1.cross) {
                    leftServo.setTargetRotation(0);
                    //rightServo.setTargetRotation(0);
                }

                if (gamepad1.triangle) {
                    leftServo.setKP(leftServo.getKP() + 0.001);
                    //rightServo.setKP(rightServo.getKP() + 0.001);
                }
                if (gamepad1.square) {
                    leftServo.setKP(Math.max(0, leftServo.getKP() - 0.001));
                    //rightServo.setKP(Math.max(0, leftServo.getKP() - 0.001));
                }

                if (gamepad1.right_bumper) {
                    leftServo.setKI(leftServo.getKI() + 0.0001);
                    //rightServo.setKI(rightServo.getKI() + 0.0001);
                }
                if (gamepad1.left_bumper) {
                    leftServo.setKI(Math.max(0, leftServo.getKI() - 0.0001));
                    //rightServo.setKI(Math.max(0, rightServo.getKI() - 0.0001));
                }

                if (gamepad1.touchpad) {
                    leftServo.setKP(0.015);
                    leftServo.setKI(0.0005);
                    leftServo.setKD(0.0025);
                    leftServo.resetPID();
//                    rightServo.setKP(0.015);
//                    rightServo.setKI(0.0005);
//                    rightServo.setKD(0.0025);
//                    rightServo.resetPID();
                }

                telemetry.addData("Starting angle", leftServo.STARTPOS);
                telemetry.addLine(leftServo.log());
                telemetry.addData("NTRY", leftServo.ntry);
//                telemetry.addData("Starting angle", rightServo.STARTPOS);
//                telemetry.addLine(rightServo.log());
//                telemetry.addData("NTRY", rightServo.ntry);
                telemetry.update();
            }
        }
    }
}