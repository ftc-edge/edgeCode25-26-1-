package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.util.Context;
import org.firstinspires.ftc.teamcode.tests.SensorGoBildaPinpointExample;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yaiba.BODY;

import java.io.IOException;

@Autonomous
public class YAIBA extends LinearOpMode {
    private BODY body;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // TFLite wrapper
    private BODY yaiba;
    private SensorGoBildaPinpointExample odo;

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2) Initialize TFLite BODY (YAIBA)
        try {
            // BODY expects an Android Context — LinearOpMode is a Context
            yaiba = new BODY(hardwareMap.appContext);
        } catch (IOException e) {
            telemetry.addData("YAIBA", "Failed to load model: " + e.getMessage());
            telemetry.update();
            // If model fails to load, abort safely
            requestOpModeStop();
            return;
        }

        telemetry.addLine("Press PLAY to start YAIBA autonomous");
        telemetry.update();
        waitForStart();

        // --- Autonomous loop ---
        // This demo will run until stop requested. In a real match, use a time limit or state machine.
        while (opModeIsActive()) {

            // --------- Get your robot state (agentX, agentY) and desired target (targetX, targetY) ----------
            // IMPORTANT: Replace the placeholders below with your odometry/localization output.
            // The BODY wrapper expects the same ordering you used during training:
            // our example puts agent coords into obs_0[0..1] and target into obs_1[0..1].
            float agentX = getAgentX();   // TODO: implement from odometry or sensors
            float agentY = getAgentY();   // TODO
            float targetX = getTargetX(); // TODO: e.g., a waypoint or vision result
            float targetY = getTargetY(); // TODO

            // If you don't have odometry yet, use a simple demo target:
            // float agentX = 0f, agentY = 0f, targetX = 3f, targetY = 0f;

            // 3) Run inference (deterministic head)
            float[] action = yaiba.runDeterministic(agentX, agentY, targetX, targetY);
            float forward = clamp(action[0], -1f, 1f);    // [-1, 1]
            float strafe  = clamp(action[1], -1f, 1f);    // [-1, 1]

            // 4) Map normalized actions [-1,1] to motor powers [-1,1]
            // Mecanum mapping (no rotation):
            // frontLeft  = forward + strafe
            // frontRight = forward - strafe
            // backLeft   = forward - strafe
            // backRight  = forward + strafe
            double fl = forward + strafe;
            double fr = forward - strafe;
            double bl = forward - strafe;
            double br = forward + strafe;

            // Normalise in case any value is outside [-1,1]
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            // 5) Apply powers to motors
            frontLeft.setPower(fl);
            frontRight.setPower(fr);
            backLeft.setPower(bl);
            backRight.setPower(br);

            // Telemetry
            telemetry.addData("agent", "(%.2f, %.2f)", agentX, agentY);
            telemetry.addData("target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("action", "fwd=%.2f str=%.2f", forward, strafe);
            telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
            telemetry.update();

        }

        // Clean up
        yaiba.close();
    }

    // ---- Helper functions / placeholders ----

    private float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Replace these with your odometry or pose estimate code (encoders, IMU, external)
    private float getAgentX() {
        return odo.getXPose();
    }
    private float getAgentY() {
        return odo.getYPose();
    }
    private float getTargetX() {
        // Example: a fixed waypoint 3 meters ahead
        return 3f;
    }
    private float getTargetY() {
        return 0f;
    }
}

