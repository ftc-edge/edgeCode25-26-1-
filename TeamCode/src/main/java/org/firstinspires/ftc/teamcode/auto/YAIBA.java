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

    private float targetX = 0f;
    private float targetY = 0f;

    private double fl;
    private double fr;
    private double bl;
    private double br;
    private float forward;
    private float strafe;
    private int cooldownCounter = 0;

    public boolean ifMotorFL = true;
    public boolean ifMotorFR = true;
    public boolean ifMotorBL = true;
    public boolean ifMotorBR = true;

    public static final float DISTANCE_TOLERANCE = 0;

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
        odo.resetPosAndIMU();

        waitForStart();

        // --- Autonomous loop ---
        // This demo will run until stop requested. In a real match, use a time limit or state machine.
        while (opModeIsActive()) {
            odo.update();
            Pose2D currentPose = odo.getPosition();
            // --------- Get your robot state (agentX, agentY) and desired target (targetX, targetY) ----------
            // IMPORTANT: Replace the placeholders below with your odometry/localization output.
            // The BODY wrapper expects the same ordering you used during training:
            // our example puts agent coords into obs_0[0..1] and target into obs_1[0..1].
            float agentX = getAgentX();   // TODO: implement from odometry or sensors
            float agentY = getAgentY();   // TODO
            checkTarget();

            //telemetry.addData("Yaiba Outputs", yaiba.runDeterministic(agentX, agentY, targetX, targetY));
            // 3) Run inference (deterministic head)

//            if(cooldownCounter == 0) {
                actions = yaiba.runDeterministic(agentX, agentY, targetX, targetY);
//                cooldownCounter = 10;
//            }else{
//                cooldownCounter--;
//            }

            strafe = actions[0];    // [-1, 1]
            forward = actions[1];    // [-1, 1]

            // 4) Map normalized actions [-1,1] to motor powers [-1,1]
            // Mecanum mapping (no rotation):
            // frontLeft  = forward + strafe
            // frontRight = forward - strafe
            // backLeft   = forward - strafe
            // backRight  = forward + strafe
            fl = forward + strafe;
            fr = forward - strafe;
            bl = forward - strafe;
            br = forward + strafe;



            // Normalise in case any value is outside [-1,1]
            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
            fl /= max; fr /= max; bl /= max; br /= max;

            if(Math.hypot(targetX - agentX, targetY - agentY) > DISTANCE_TOLERANCE) {
                // 5) Apply powers to motors
                    frontLeft.setPower(fl);
                    frontRight.setPower(fr);
                    backLeft.setPower(bl);
                    backRight.setPower(br);
            }else{
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }


            // Telemetry
            telemetry.addData("agent", "(%.2f, %.2f)", agentX, agentY);
            telemetry.addData("target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Forward (Original)", actions[1]);
            telemetry.addData("Forward (Derivative)", forward);
            telemetry.addData("Strafe (Original", strafe);
            telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
            telemetry.addData("cooldown counter", cooldownCounter);
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
        targetX = gamepad1.left_stick_x * 91.44f;
        targetY = gamepad1.left_stick_y * -91.44f;
    }
}

