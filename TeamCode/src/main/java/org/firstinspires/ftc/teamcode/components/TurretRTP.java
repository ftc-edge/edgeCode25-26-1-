package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Config
public class TurretRTP{

    // Hardware
    public CRServo turretServo1;
    public CRServo turretServo2;
    public DcMotorEx turretEncoder;

    // Servo direction configuration
    public static boolean INVERT_SERVO2 = false;
    public static boolean INVERT_ENCODER = false;  // Try false if turret goes wrong way

    // FORBIDDEN ZONE CONFIGURATION
    public static double FORBIDDEN_START_DEGREES = 115;
    public static double FORBIDDEN_END_DEGREES = 145;

    // PID Controller constants - Tunable via FTC Dashboard
    public static double kP = 0.01;   // Tuned down from 0.015
    public static double kI = 0.003; // Tuned down from 0.0005
    public static double kD = 0.0005;  // Tuned down from 0.002

    // Control variables
    public double targetPosition = 0.0;  // In encoder ticks
    public double previousError = 0.0;
    public double integralSum = 0.0;
    public ElapsedTime timer = new ElapsedTime();
    public boolean firstPIDCall = true;  // Track first PID call to avoid bad dt

    // Encoder settings
    public static double TICKS_PER_REVOLUTION = 8192 * (130.0/24.0);
    public double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;
    public double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    // Deadzone
    public static double DEADZONE_TICKS = 100;  // About 4.4 degrees

    // Constructor for use in other classes
    public TurretRTP(HardwareMap hardwareMap) {
        // Initialize hardware
        turretServo1 = hardwareMap.get(CRServo.class, "leftServo");
        turretServo2 = hardwareMap.get(CRServo.class, "rightServo");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "BLmotor");

        // Configure encoder - DON'T reset so we maintain position
        // If you need to reset, do it manually or set a flag
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize target to current position
        targetPosition = turretEncoder.getCurrentPosition();

        // Start timer
        timer.reset();
        firstPIDCall = true;
    }

    /**
     * Reset the encoder to zero at current position
     */
    public void resetEncoder() {
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
        integralSum = 0;
        previousError = 0;
        firstPIDCall = true;
    }

    /**
     * Get safe target - snap to boundary if in forbidden zone
     */
    public double getSafeTarget(double desiredDegrees) {
        if (isInForbiddenZone(desiredDegrees)) {
            desiredDegrees = snapToNearestBoundary(desiredDegrees);
        }
        return desiredDegrees * TICKS_PER_DEGREE;
    }

    /**
     * Smart PID that calculates error considering routing around forbidden zone
     */
    public double calculateSmartPID(double currentDegrees, double targetDegrees) {
        // Calculate both possible paths
        double clockwiseDistance = calculateClockwiseDistance(currentDegrees, targetDegrees);
        double counterClockwiseDistance = 360 - clockwiseDistance;

        // Check which paths are safe
        boolean clockwiseSafe = !pathCrossesForbiddenZoneDirection(currentDegrees, targetDegrees, true);
        boolean counterClockwiseSafe = !pathCrossesForbiddenZoneDirection(currentDegrees, targetDegrees, false);

        // Choose the error (path) to use
        double error;
        if (clockwiseSafe && counterClockwiseSafe) {
            // Both safe - take shorter path
            if (clockwiseDistance < counterClockwiseDistance) {
                error = clockwiseDistance; // Positive = clockwise
            } else {
                error = -counterClockwiseDistance; // Negative = counter-clockwise
            }
        } else if (clockwiseSafe) {
            error = clockwiseDistance; // Must go clockwise
        } else if (counterClockwiseSafe) {
            error = -counterClockwiseDistance; // Must go counter-clockwise
        } else {
            // Neither safe (shouldn't happen) - stop
            return 0;
        }

        // Convert error to ticks
        double errorTicks = error * TICKS_PER_DEGREE;

        // Apply deadzone
        if (Math.abs(errorTicks) < DEADZONE_TICKS) {
            integralSum = 0;
            previousError = 0;
            return 0.0;
        }

        // Get time delta - handle first call specially
        double dt;
        if (firstPIDCall) {
            dt = 0.02; // Assume 50Hz on first call
            firstPIDCall = false;
        } else {
            dt = timer.seconds();
        }
        timer.reset();

        // Clamp dt to reasonable values (20ms to 200ms)
        dt = Range.clip(dt, 0.001, 0.2);

        // PID calculations
        double P = kP * errorTicks;

        // Integral (with anti-windup)
        integralSum += errorTicks * dt;
        integralSum = Range.clip(integralSum, -100.0, 100.0);
        double I = kI * integralSum;

        // Derivative
        double derivative = (errorTicks - previousError) / dt;
        double D = kD * derivative;
        previousError = errorTicks;

        double output = P + I + D;
        return Range.clip(output, -1.0, 1.0);
    }

    /**
     * Calculate clockwise distance from start to end (always positive)
     */
    public double calculateClockwiseDistance(double start, double end) {
        double distance = end - start;
        if (distance < 0) distance += 360;
        return distance;
    }

    /**
     * Check if path from start to end crosses forbidden zone in specified direction
     */
    public boolean pathCrossesForbiddenZoneDirection(double startDeg, double endDeg, boolean clockwise) {
        startDeg = normalizeAngle(startDeg);
        endDeg = normalizeAngle(endDeg);

        if (clockwise) {
            // Going clockwise: increasing angles with wraparound at 360
            if (startDeg < endDeg) {
                // Simple case: 50° → 200° (no wraparound)
                return (FORBIDDEN_START_DEGREES > startDeg && FORBIDDEN_START_DEGREES < endDeg) ||
                        (FORBIDDEN_END_DEGREES > startDeg && FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                // Wraparound case: 270° → 90° (wraps through 0/360)
                return (FORBIDDEN_START_DEGREES > startDeg) || (FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            }
        } else {
            // Going counter-clockwise: decreasing angles with wraparound at 0
            if (startDeg > endDeg) {
                // Simple case: 200° → 50° (no wraparound)
                return (FORBIDDEN_START_DEGREES < startDeg && FORBIDDEN_START_DEGREES > endDeg) ||
                        (FORBIDDEN_END_DEGREES < startDeg && FORBIDDEN_END_DEGREES > endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                // Wraparound case: 90° → 270° counter-clockwise (wraps through 0/360)
                return (FORBIDDEN_END_DEGREES < startDeg) || (FORBIDDEN_START_DEGREES > endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            }
        }
    }

    /**
     * Check if path from start to end crosses forbidden zone
     */
    public boolean pathCrossesForbiddenZone(double startDeg, double endDeg) {
        startDeg = normalizeAngle(startDeg);
        endDeg = normalizeAngle(endDeg);

        if (endDeg > startDeg) {
            return (startDeg < FORBIDDEN_START_DEGREES && endDeg > FORBIDDEN_START_DEGREES) ||
                    (startDeg < FORBIDDEN_END_DEGREES && endDeg > FORBIDDEN_END_DEGREES) ||
                    (startDeg < FORBIDDEN_START_DEGREES && endDeg > FORBIDDEN_END_DEGREES);
        } else if (endDeg < startDeg) {
            return !((endDeg < FORBIDDEN_START_DEGREES && startDeg < FORBIDDEN_START_DEGREES) ||
                    (endDeg > FORBIDDEN_END_DEGREES && startDeg > FORBIDDEN_END_DEGREES));
        }

        return false;
    }

    /**
     * Get human-readable route info
     */
    public String getRouteInfo(double current, double target) {
        double clockwise = calculateClockwiseDistance(current, target);
        double counterClockwise = 360 - clockwise;

        boolean cwSafe = !pathCrossesForbiddenZoneDirection(current, target, true);
        boolean ccwSafe = !pathCrossesForbiddenZoneDirection(current, target, false);

        if (Math.abs(clockwise) < 2) return "At target";

        if (cwSafe && ccwSafe) {
            if (clockwise < counterClockwise) {
                return String.format("CW %.0f° (shorter)", clockwise);
            } else {
                return String.format("CCW %.0f° (shorter)", counterClockwise);
            }
        } else if (cwSafe && !ccwSafe) {
            return String.format("CW %.0f° (CCW blocked)", clockwise);
        } else if (ccwSafe && !cwSafe) {
            return String.format("CCW %.0f° (CW blocked)", counterClockwise);
        } else {
            return "No safe path!";
        }
    }

    /**
     * Snap an angle in forbidden zone to nearest safe boundary
     */
    public double snapToNearestBoundary(double degrees) {
        degrees = normalizeAngle(degrees);

        if (!isInForbiddenZone(degrees)) {
            return degrees;
        }

        double distToStart = degrees - FORBIDDEN_START_DEGREES;
        double distToEnd = FORBIDDEN_END_DEGREES - degrees;

        return (distToStart < distToEnd) ? FORBIDDEN_START_DEGREES - 1 : FORBIDDEN_END_DEGREES + 1;
    }

    /**
     * Normalize angle to 0-360
     */
    public double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return degrees;
    }

    /**
     * Check if in forbidden zone
     */
    public boolean isInForbiddenZone(double degrees) {
        degrees = normalizeAngle(degrees);
        return degrees >= FORBIDDEN_START_DEGREES && degrees <= FORBIDDEN_END_DEGREES;
    }

    /**
     * Check if near forbidden zone
     */
    public boolean isNearForbiddenZone(double degrees, double margin) {
        degrees = normalizeAngle(degrees);
        return (degrees >= FORBIDDEN_START_DEGREES - margin && degrees < FORBIDDEN_START_DEGREES) ||
                (degrees > FORBIDDEN_END_DEGREES && degrees <= FORBIDDEN_END_DEGREES + margin);
    }

    /**
     * Convert ticks to degrees (with optional inversion)
     */
    public double ticksToDegrees(double ticks) {
        double degrees = ticks * DEGREES_PER_TICK;
        // Invert if encoder direction is reversed
        if (INVERT_ENCODER) {
            degrees = -degrees;
        }
        return degrees;
    }

    /**
     * Get current turret angle in degrees
     */
    public double getCurrentTurretAngle() {
        return normalizeAngle(ticksToDegrees(turretEncoder.getCurrentPosition()));
    }

    /**
     * Set power to both servos
     */
    public void setTurretPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(INVERT_SERVO2 ? -power : power);
    }

    public void stop() {
        turretServo1.setPower(0);
        turretServo2.setPower(0);
    }
}