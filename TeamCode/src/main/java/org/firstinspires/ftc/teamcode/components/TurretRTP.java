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
    public final boolean INVERT_SERVO2 = false;

    // FORBIDDEN ZONE CONFIGURATION
    public final double FORBIDDEN_START_DEGREES = 115;
    public final double FORBIDDEN_END_DEGREES = 145;

    // PID Controller constants
    public static double kP = 0.003;
    public static double kI = 0.0001;
    public static double kD = 0.0002;

    // Control variables
    public double targetPosition = 0.0;  // In encoder ticks
    public double previousError = 0.0;
    public double integralSum = 0.0;
    public ElapsedTime timer = new ElapsedTime();

    // Encoder settings
    public final double TICKS_PER_REVOLUTION = 8192 * ((double) 130 /24);
    public final double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;
    public final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    // Deadzone
    public final double DEADZONE_TICKS = TICKS_PER_DEGREE * 2.0;
    
    public TurretRTP(HardwareMap hardwareMap){
        turretServo1 = hardwareMap.get(CRServo.class, "leftServo");
        turretServo2 = hardwareMap.get(CRServo.class, "rightServo");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "BLmotor");

        // Configure encoder
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize target to current position
        targetPosition = turretEncoder.getCurrentPosition();
    }

//    @Override
//    public void init() {
//        // Initialize hardware
//        turretServo1 = hardwareMap.get(CRServo.class, "leftServo");
//        turretServo2 = hardwareMap.get(CRServo.class, "rightServo");
//        turretEncoder = hardwareMap.get(DcMotorEx.class, "BLmotor");
//
//        // Configure encoder
//        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//
//        // Initialize target to current position
//        targetPosition = turretEncoder.getCurrentPosition();
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.addData("Forbidden Zone", "%.1f° to %.1f°", FORBIDDEN_START_DEGREES, FORBIDDEN_END_DEGREES);
//        telemetry.addData("Current", "%.1f°", ticksToDegrees(turretEncoder.getCurrentPosition()));
//        telemetry.update();
//    }

//    @Override
//    public void loop() {
//        // Get current encoder position
//        int currentPosition = turretEncoder.getCurrentPosition();
//        double currentDegrees = normalizeAngle(ticksToDegrees(currentPosition));
//
//        // Emergency stop if turret enters forbidden zone
//        if (isInForbiddenZone(currentDegrees)) {
//            turretServo1.setPower(0);
//            turretServo2.setPower(0);
//            telemetry.addData("⚠️ WARNING", "IN FORBIDDEN ZONE!");
//            telemetry.addData("Current", "%.1f°", currentDegrees);
//            telemetry.addData("", "Manually move out or press A to reset");
//            telemetry.update();
//            return;
//        }
//
//        // Control input - Left stick X adjusts target
//        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
//            double adjustment = gamepad1.left_stick_x * 3.0; // degrees per frame
//            double currentTargetDegrees = normalizeAngle(ticksToDegrees(targetPosition));
//            double newTargetDegrees = currentTargetDegrees + adjustment;
//            newTargetDegrees = normalizeAngle(newTargetDegrees);
//
//            // If new target would be in forbidden zone, snap to nearest boundary
//            if (isInForbiddenZone(newTargetDegrees)) {
//                newTargetDegrees = snapToNearestBoundary(newTargetDegrees);
//            }
//
//            targetPosition = newTargetDegrees * TICKS_PER_DEGREE;
//        }
//
//        // D-pad for preset angles
//        if (gamepad1.dpad_left) {
//            targetPosition = getSafeTarget(90);
//        } else if (gamepad1.dpad_right) {
//            targetPosition = getSafeTarget(180);
//        } else if (gamepad1.dpad_up) {
//            targetPosition = getSafeTarget(270);
//        } else if (gamepad1.dpad_down) {
//            targetPosition = getSafeTarget(0);
//        }
//
//        // A button to reset encoder
//        if (gamepad1.a) {
//            turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//            turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
//            targetPosition = 0;
//            integralSum = 0;
//            previousError = 0;
//        }
//
//        // Calculate power using smart PID that routes around forbidden zone
//        double power = calculateSmartPID(currentDegrees, normalizeAngle(ticksToDegrees(targetPosition)));
//
//        // Apply power to servos
//        setTurretPower(power);
//
//        // Telemetry
//        double targetDegrees = normalizeAngle(ticksToDegrees(targetPosition));
//        String routeInfo = getRouteInfo(currentDegrees, targetDegrees);
//
//        telemetry.addData("Target", "%.1f°", targetDegrees);
//        telemetry.addData("Current", "%.1f°", currentDegrees);
//        telemetry.addData("Route", routeInfo);
//        telemetry.addData("Power", "%.3f", power);
//        telemetry.addData("Forbidden", "%.1f° to %.1f°", FORBIDDEN_START_DEGREES, FORBIDDEN_END_DEGREES);
//
//        if (isNearForbiddenZone(currentDegrees, 10)) {
//            telemetry.addData("⚠️", "Near forbidden zone!");
//        }
//
//        telemetry.addData("", "");
//        telemetry.addData("Controls", "Stick: Adjust | D-Pad: Presets | A: Reset");
//        telemetry.update();
//    }

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

        // PID calculations
        double P = kP * errorTicks;

        double dt = timer.seconds();
        timer.reset();
        integralSum += errorTicks * dt;
        integralSum = Range.clip(integralSum, -100.0, 100.0);
        double I = kI * integralSum;

        double derivative = (dt > 0) ? (errorTicks - previousError) / dt : 0;
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
     * @param startDeg Starting angle
     * @param endDeg Ending angle
     * @param clockwise True for clockwise, false for counter-clockwise
     */
    public boolean pathCrossesForbiddenZoneDirection(double startDeg, double endDeg, boolean clockwise) {
        startDeg = normalizeAngle(startDeg);
        endDeg = normalizeAngle(endDeg);

        if (clockwise) {
            // Going clockwise: increasing angles with wraparound at 360
            if (startDeg < endDeg) {
                // Simple case: 50° → 200° (no wraparound)
                // Check if forbidden zone (115-145) is between start and end
                return (FORBIDDEN_START_DEGREES > startDeg && FORBIDDEN_START_DEGREES < endDeg) ||
                        (FORBIDDEN_END_DEGREES > startDeg && FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                // Wraparound case: 270° → 90° (wraps through 0/360)
                // Path is: 270° → 360°/0° → 90°
                // Check if forbidden zone is in either segment
                return (FORBIDDEN_START_DEGREES > startDeg) || (FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            }
        } else {
            // Going counter-clockwise: decreasing angles with wraparound at 0
            if (startDeg > endDeg) {
                // Simple case: 200° → 50° (no wraparound)
                // Check if forbidden zone is between start and end
                return (FORBIDDEN_START_DEGREES < startDeg && FORBIDDEN_START_DEGREES > endDeg) ||
                        (FORBIDDEN_END_DEGREES < startDeg && FORBIDDEN_END_DEGREES > endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                // Wraparound case: 90° → 270° counter-clockwise (wraps through 0/360)
                // Path is: 90° → 0°/360° → 270°
                // Check if forbidden zone is in either segment
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

        // If we're going clockwise
        if (endDeg > startDeg) {
            // Check if forbidden zone is between start and end
            return (startDeg < FORBIDDEN_START_DEGREES && endDeg > FORBIDDEN_START_DEGREES) ||
                    (startDeg < FORBIDDEN_END_DEGREES && endDeg > FORBIDDEN_END_DEGREES) ||
                    (startDeg < FORBIDDEN_START_DEGREES && endDeg > FORBIDDEN_END_DEGREES);
        } else if (endDeg < startDeg) {
            // Going counter-clockwise OR wrapping around through 0
            // Check if we pass through 0 and if forbidden zone is in the wrapped path
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
            // Both safe - show which is shorter
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
     * Convert ticks to degrees
     */
    public double ticksToDegrees(double ticks) {
        return ticks * DEGREES_PER_TICK;
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