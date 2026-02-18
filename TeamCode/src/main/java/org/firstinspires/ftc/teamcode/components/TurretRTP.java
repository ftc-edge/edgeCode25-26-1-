package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
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
    public static boolean INVERT_ENCODER = false;

    // ANGLE INVERSION: Set true if your 45° is showing as 315°
    // This flips the entire angle coordinate system
    public static boolean INVERT_ANGLES = true;

    // FORBIDDEN ZONE - in ENCODER space (physical position where wires tangle)
    public static double FORBIDDEN_START_DEGREES = 115;
    public static double FORBIDDEN_END_DEGREES = 145;

    // PID Controller constants - Tunable via FTC Dashboard
    public static double kP = 0.015;
    public static double kI = 0.0005;
    public static double kD = 0.002;

    // Control variables
    public double targetPosition = 0.0;  // In encoder ticks
    public double previousError = 0.0;
    public double integralSum = 0.0;
    public ElapsedTime timer = new ElapsedTime();
    public boolean firstPIDCall = true;

    // Encoder settings
    public static double TICKS_PER_REVOLUTION = 8192 * (130/24);
    public double DEGREES_PER_TICK = 360.0 / TICKS_PER_REVOLUTION;
    public double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    // Deadzone
    public static double DEADZONE_TICKS = 50;

    public TurretRTP(HardwareMap hardwareMap) {
        turretServo1 = hardwareMap.get(CRServo.class, "leftServo");
        turretServo2 = hardwareMap.get(CRServo.class, "rightServo");
        turretEncoder = hardwareMap.get(DcMotorEx.class, "BLmotor");

        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = turretEncoder.getCurrentPosition();

        timer.reset();
        firstPIDCall = true;
    }

    /**
     * Convert user-space degrees to encoder-space degrees
     * User space: what you think of as 45° is 45°
     * Encoder space: what the encoder actually reads
     */
    public double userToEncoderDegrees(double userDegrees) {
        if (INVERT_ANGLES) {
            return 360 - normalizeAngle(userDegrees);
        }
        return userDegrees;
    }

    /**
     * Convert encoder-space degrees to user-space degrees
     */
    public double encoderToUserDegrees(double encoderDegrees) {
        if (INVERT_ANGLES) {
            return 360 - normalizeAngle(encoderDegrees);
        }
        return encoderDegrees;
    }

    /**
     * Convert ticks to encoder-space degrees (NO inversion here, raw reading)
     */
    public double ticksToEncoderDegrees(double ticks) {
        if (INVERT_ENCODER) {
            ticks = -ticks;
        }
        return ticks * DEGREES_PER_TICK;
    }

    /**
     * Get current turret angle in USER space (what you expect to see)
     */
    public double getCurrentTurretAngleUser() {
        double encoderDegrees = ticksToEncoderDegrees(turretEncoder.getCurrentPosition());
        return normalizeAngle(encoderToUserDegrees(encoderDegrees));
    }

    /**
     * Get current turret angle in ENCODER space (for internal calculations)
     */
    public double getCurrentTurretAngleEncoder() {
        return normalizeAngle(ticksToEncoderDegrees(turretEncoder.getCurrentPosition()));
    }

    /**
     * Set target in USER space degrees
     * Converts to encoder space, checks forbidden zone, returns ticks
     */
    public double getSafeTargetFromUser(double userDegrees) {
        // Convert to encoder space
        double encoderDegrees = userToEncoderDegrees(userDegrees);

        // Check forbidden zone (in encoder space)
        if (isInForbiddenZone(encoderDegrees)) {
            encoderDegrees = snapToNearestBoundary(encoderDegrees);
        }

        return encoderDegrees * TICKS_PER_DEGREE;
    }

    /**
     * Smart PID - works in ENCODER space
     */
    public double calculateSmartPID(double currentEncoderDegrees, double targetEncoderDegrees) {
        double clockwiseDistance = calculateClockwiseDistance(currentEncoderDegrees, targetEncoderDegrees);
        double counterClockwiseDistance = 360 - clockwiseDistance;

        boolean clockwiseSafe = !pathCrossesForbiddenZoneDirection(currentEncoderDegrees, targetEncoderDegrees, true);
        boolean counterClockwiseSafe = !pathCrossesForbiddenZoneDirection(currentEncoderDegrees, targetEncoderDegrees, false);

        double error;
        if (clockwiseSafe && counterClockwiseSafe) {
            if (clockwiseDistance < counterClockwiseDistance) {
                error = clockwiseDistance;
            } else {
                error = -counterClockwiseDistance;
            }
        } else if (clockwiseSafe) {
            error = clockwiseDistance;
        } else if (counterClockwiseSafe) {
            error = -counterClockwiseDistance;
        } else {
            return 0;
        }

        double errorTicks = error * TICKS_PER_DEGREE;

        if (Math.abs(errorTicks) < DEADZONE_TICKS) {
            integralSum = 0;
            previousError = 0;
            return 0.0;
        }

        double dt;
        if (firstPIDCall) {
            dt = 0.02;
            firstPIDCall = false;
        } else {
            dt = timer.seconds();
        }
        timer.reset();
        dt = Range.clip(dt, 0.02, 0.2);

        double P = kP * errorTicks;

        integralSum += errorTicks * dt;
        integralSum = Range.clip(integralSum, -100.0, 100.0);
        double I = kI * integralSum;

        double derivative = (errorTicks - previousError) / dt;
        double D = kD * derivative;
        previousError = errorTicks;

        double output = P + I + D;
        return Range.clip(output, -1.0, 1.0);
    }

    public double calculateClockwiseDistance(double start, double end) {
        double distance = end - start;
        if (distance < 0) distance += 360;
        return distance;
    }

    public boolean pathCrossesForbiddenZoneDirection(double startDeg, double endDeg, boolean clockwise) {
        startDeg = normalizeAngle(startDeg);
        endDeg = normalizeAngle(endDeg);

        if (clockwise) {
            if (startDeg < endDeg) {
                return (FORBIDDEN_START_DEGREES > startDeg && FORBIDDEN_START_DEGREES < endDeg) ||
                        (FORBIDDEN_END_DEGREES > startDeg && FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                return (FORBIDDEN_START_DEGREES > startDeg) || (FORBIDDEN_END_DEGREES < endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            }
        } else {
            if (startDeg > endDeg) {
                return (FORBIDDEN_START_DEGREES < startDeg && FORBIDDEN_START_DEGREES > endDeg) ||
                        (FORBIDDEN_END_DEGREES < startDeg && FORBIDDEN_END_DEGREES > endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            } else {
                return (FORBIDDEN_END_DEGREES < startDeg) || (FORBIDDEN_START_DEGREES > endDeg) ||
                        (startDeg >= FORBIDDEN_START_DEGREES && startDeg <= FORBIDDEN_END_DEGREES) ||
                        (endDeg >= FORBIDDEN_START_DEGREES && endDeg <= FORBIDDEN_END_DEGREES);
            }
        }
    }

    public String getRouteInfo(double currentEncoderDeg, double targetEncoderDeg) {
        double clockwise = calculateClockwiseDistance(currentEncoderDeg, targetEncoderDeg);
        double counterClockwise = 360 - clockwise;

        boolean cwSafe = !pathCrossesForbiddenZoneDirection(currentEncoderDeg, targetEncoderDeg, true);
        boolean ccwSafe = !pathCrossesForbiddenZoneDirection(currentEncoderDeg, targetEncoderDeg, false);

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

    public double snapToNearestBoundary(double encoderDegrees) {
        encoderDegrees = normalizeAngle(encoderDegrees);

        if (!isInForbiddenZone(encoderDegrees)) {
            return encoderDegrees;
        }

        double distToStart = encoderDegrees - FORBIDDEN_START_DEGREES;
        double distToEnd = FORBIDDEN_END_DEGREES - encoderDegrees;

        return (distToStart < distToEnd) ? FORBIDDEN_START_DEGREES - 1 : FORBIDDEN_END_DEGREES + 1;
    }

    public double normalizeAngle(double degrees) {
        degrees = degrees % 360;
        if (degrees < 0) degrees += 360;
        return degrees;
    }

    public boolean isInForbiddenZone(double encoderDegrees) {
        encoderDegrees = normalizeAngle(encoderDegrees);
        return encoderDegrees >= FORBIDDEN_START_DEGREES && encoderDegrees <= FORBIDDEN_END_DEGREES;
    }

    public void setTurretPower(double power) {
        turretServo1.setPower(power);
        turretServo2.setPower(INVERT_SERVO2 ? -power : power);
    }

    public void stop() {
        turretServo1.setPower(0);
        turretServo2.setPower(0);
    }

    public void resetEncoder() {
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
        integralSum = 0;
        previousError = 0;
        firstPIDCall = true;
    }
}