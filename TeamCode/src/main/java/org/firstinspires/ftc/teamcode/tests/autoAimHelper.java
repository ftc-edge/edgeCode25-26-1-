package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretRTP;
import org.firstinspires.ftc.teamcode.components.TurretSpin;

@TeleOp
public class autoAimHelper extends OpMode {
    TurretSpin turretSpin;

    TurretRTP rtp;

    private float turretPos = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpin = new TurretSpin(hardwareMap);
        rtp = new TurretRTP(hardwareMap);
    }

    @Override
    public void loop() {
        // Get current encoder position
        int currentPosition = rtp.turretEncoder.getCurrentPosition();
        double currentDegrees = rtp.normalizeAngle(rtp.ticksToDegrees(currentPosition));

        // Emergency stop if turret enters forbidden zone
        if (rtp.isInForbiddenZone(currentDegrees)) {
            rtp.turretServo1.setPower(0);
            rtp.turretServo2.setPower(0);
            telemetry.addData("⚠️ WARNING", "IN FORBIDDEN ZONE!");
            telemetry.addData("Current", "%.1f°", currentDegrees);
            telemetry.addData("", "Manually move out or press A to reset");
            telemetry.update();
            return;
        }

        // Control input - Left stick X adjusts target
        if (Math.abs(gamepad1.left_stick_x) > 0.1) {
            double adjustment = gamepad1.left_stick_x * 3.0; // degrees per frame
            double currentTargetDegrees = rtp.normalizeAngle(rtp.targetPosition * rtp.DEGREES_PER_TICK);
            double newTargetDegrees = currentTargetDegrees + adjustment;
            newTargetDegrees = rtp.normalizeAngle(newTargetDegrees);

            // If new target would be in forbidden zone, snap to nearest boundary
            if (rtp.isInForbiddenZone(newTargetDegrees)) {
                newTargetDegrees = rtp.snapToNearestBoundary(newTargetDegrees);
            }

            rtp.targetPosition = newTargetDegrees * rtp.TICKS_PER_DEGREE;
        }

        // D-pad for preset angles
        if (gamepad1.dpad_left) {
            rtp.targetPosition = rtp.getSafeTarget(90);
        } else if (gamepad1.dpad_right) {
            rtp.targetPosition = rtp.getSafeTarget(180);
        } else if (gamepad1.dpad_up) {
            rtp.targetPosition = rtp.getSafeTarget(270);
        } else if (gamepad1.dpad_down) {
            rtp.targetPosition = rtp.getSafeTarget(0);
        }

        // A button to reset encoder
        if (gamepad1.a) {
            rtp.turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            rtp.turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            rtp.targetPosition = 0;
            rtp.integralSum = 0;
            rtp.previousError = 0;
        }

        // Calculate power using smart PID that routes around forbidden zone
        double power = rtp.calculateSmartPID(currentDegrees, rtp.normalizeAngle(rtp.ticksToDegrees(rtp.targetPosition)));

        // Apply power to servos
        rtp.setTurretPower(power);

        // Telemetry
        double targetDegrees = rtp.normalizeAngle(rtp.ticksToDegrees(rtp.targetPosition));
        String routeInfo = rtp.getRouteInfo(currentDegrees, targetDegrees);

        telemetry.addData("Target", "%.1f°", targetDegrees);
        telemetry.addData("Current", "%.1f°", currentDegrees);
        telemetry.addData("Route", routeInfo);
        telemetry.addData("Power", "%.3f", power);
        telemetry.addData("Forbidden", "%.1f° to %.1f°",rtp.FORBIDDEN_START_DEGREES, rtp.FORBIDDEN_END_DEGREES);

        if (rtp.isNearForbiddenZone(currentDegrees, 10)) {
            telemetry.addData("⚠️", "Near forbidden zone!");
        }

        telemetry.addData("", "");
        telemetry.addData("Controls", "Stick: Adjust | D-Pad: Presets | A: Reset");
        telemetry.update();
    }


}
