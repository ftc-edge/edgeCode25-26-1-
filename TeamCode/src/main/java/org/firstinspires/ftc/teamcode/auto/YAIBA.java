package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.util.Context;
import org.firstinspires.ftc.teamcode.tests.SensorGoBildaPinpointExample;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yaiba.BODY;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.teamcode.yaiba.ModelInputMapper;

import java.io.IOException;

@TeleOp
public class YAIBA extends LinearOpMode {
    private BODY body;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // TFLite wrapper
    private BODY yaiba;
    private GoBildaPinpointDriver odo;

    private float[] actions;

    private float targetX = 15f;
    private float targetY = 0f;

    @Override
    public void runOpMode() {
        // 2) Initialize TFLite BODY (YAIBA)
        yaiba = BODY.create(hardwareMap.appContext);
        if (yaiba == null) {
            telemetry.addData("MODEL", "Failed to load BODY.tflite");
            telemetry.addLine("Check: app/src/main/assets/BODY.tflite");
            telemetry.update();
            // Keep going but make sure any inference calls are guarded (yaiba != null).
        } else {
            telemetry.addData("MODEL", "Loaded successfully");
            telemetry.update();
        }

        telemetry.addLine("Press PLAY to start YAIBA autonomous");
        telemetry.update();

        frontLeft  = hardwareMap.get(DcMotor.class, "FLmotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backLeft   = hardwareMap.get(DcMotor.class, "BLmotor");
        backRight  = hardwareMap.get(DcMotor.class, "BRmotor");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        //odo.resetPosAndIMU();
        Pose2D startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        odo.setPosition(startPose);

        waitForStart();

        // --- Autonomous loop ---
        // This demo will run until stop requested. In a real match, use a time limit or state machine.
        while (opModeIsActive()) {
            odo.update();
            Pose2D currentPose = new Pose2D(DistanceUnit.CM, getAgentX(), getAgentY(), AngleUnit.DEGREES, 0);
            odo.setPosition(currentPose);
            // --------- Get your robot state (agentX, agentY) and desired target (targetX, targetY) ----------
            // IMPORTANT: Replace the placeholders below with your odometry/localization output.
            // The BODY wrapper expects the same ordering you used during training:
            // our example puts agent coords into obs_0[0..1] and target into obs_1[0..1].
            float agentX = getAgentX();   // TODO: implement from odometry or sensors
            float agentY = getAgentY();   // TODO
            checkTarget();

            telemetry.addData("Yaiba Outputs", yaiba.runDeterministic(agentX, agentY, targetX, targetY));
            telemetry.update();
            // 3) Run inference (deterministic head)
            actions = yaiba.runDeterministic(agentX, agentY, targetX, targetY);
            float forward = clamp(actions[0], -1f, 1f);    // [-1, 1]
            float strafe  = clamp(actions[1], -1f, 1f);    // [-1, 1]

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
            frontLeft.setPower(0.25 * fl);
            frontRight.setPower(0.25 * fr);
            backLeft.setPower(0.25 * bl);
            backRight.setPower(0.25 *br);


            // Telemetry
            telemetry.addData("agent", "(%.2f, %.2f)", agentX, agentY);
            telemetry.addData("target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("action", "fwd=%.2f str=%.2f", forward, strafe);
            telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
            telemetry.update();

        }
            yaiba.close();
        // Clean up
    }

    // ---- Helper functions / placeholders ----

    private float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    // Replace these with your odometry or pose estimate code (encoders, IMU, external)
    private float getAgentX() {
        float x = 0;
        Pose2D pose = odo.getPosition();
        odo.update();
        x = (float)pose.getX(DistanceUnit.CM);
        return x;
    }
    private float getAgentY() {
        float y = 0;
        Pose2D pose = odo.getPosition();
        odo.update();
        y = (float)pose.getY(DistanceUnit.CM);
        return y;
    }
    private void checkTarget(){
        if(gamepad1.y){
            targetX = 121.9f;
            targetY = 121.9f;
        }
        if(gamepad1.b){
            targetX = 121.9f;
            targetY = -121.9f;
        }
        if(gamepad1.a){
            targetX = -121.9f;
            targetY = -121.9f;
        }
        if(gamepad1.x){
            targetX = -121.9f;
            targetY = 121.9f;
        }
    }
}

