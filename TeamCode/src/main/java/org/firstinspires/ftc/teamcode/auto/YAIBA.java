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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

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

    public static final float DISTANCE_TOLERANCE = 2.5f;
    private float DTT;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        yaiba = BODY.create(hardwareMap.appContext);
        if (yaiba == null) {
            telemetry.addData("MODEL", "Failed to load BODY.tflite");
            telemetry.addLine("Check: app/src/main/assets/BODY.tflite");
            telemetry.update();
        } else {
            telemetry.addData("MODEL", "Loaded successfully");
            telemetry.addData("MODEL", "Loaded successfully");
            telemetry.update();
        }

        telemetry.addLine("YAIBA Ready");
        telemetry.update();

        frontLeft  = hardwareMap.get(DcMotor.class, "FLmotor");
        frontRight = hardwareMap.get(DcMotor.class, "FRmotor");
        backLeft   = hardwareMap.get(DcMotor.class, "BLmotor");
        backRight  = hardwareMap.get(DcMotor.class, "BRmotor");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        waitForStart();

        while (opModeIsActive()) {
            odo.update();
            Pose2D currentPose = odo.getPosition();

            float agentX = getAgentX();
            float agentY = getAgentY();
            checkTarget();

            DTT = (float) Math.hypot(targetX - agentX, targetY - agentY);

            actions = yaiba.runDeterministic(agentX, agentY, targetX, targetY);

            strafe = actions[0];
            forward = actions[1];

            fl = forward + strafe;
            fr = forward - strafe;
            bl = forward - strafe;
            br = forward + strafe;



            // Normalise in case any value is outside [-1,1]
//            double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
//            fl /= max; fr /= max; bl /= max; br /= max;

            if (DTT > DISTANCE_TOLERANCE) {
                    frontLeft.setPower(fl);
                    frontRight.setPower(fr);
                    backLeft.setPower(bl);
                    backRight.setPower(br);
            } else {
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }

            // Create telemetry packet with field overlay
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay();

            double heading = currentPose.getHeading(AngleUnit.RADIANS);

            // Convert cm to inches for FTC Dashboard (uses official field frame in inches)
            double agentXInches = agentX / 2.54;
            double agentYInches = agentY / 2.54;
            double targetXInches = targetX / 2.54;
            double targetYInches = targetY / 2.54;

            // Draw target position (red circle)
            fieldOverlay.setStroke("red");
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.strokeCircle(targetXInches, targetYInches, 4);
            fieldOverlay.setFill("red");
            fieldOverlay.setAlpha(0.3);
            fieldOverlay.fillCircle(targetXInches, targetYInches, 4);

            // Draw robot position (blue circle with direction indicator)
            fieldOverlay.setAlpha(1.0);
            fieldOverlay.setStroke("blue");
            fieldOverlay.setStrokeWidth(1);
            fieldOverlay.strokeCircle(agentXInches, agentYInches, 9);
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(agentXInches, agentYInches, 9);

            // Draw direction line on robot
            double lineLength = 9;
            double lineEndX = agentXInches + lineLength * Math.cos(heading);
            double lineEndY = agentYInches + lineLength * Math.sin(heading);
            fieldOverlay.setStroke("white");
            fieldOverlay.setStrokeWidth(2);
            fieldOverlay.strokeLine(agentXInches, agentYInches, lineEndX, lineEndY);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            // Draw desired movement vector (green line showing forward/strafe direction)
            double movementMagnitude = Math.sqrt(forward * forward + strafe * strafe);
            if (movementMagnitude > 0.01) {
                // Scale the movement vector for visibility (20 inches at full power)
                double vectorScale = 20.0;
                double movementAngle = Math.atan2(forward, strafe); // Note: field uses standard trig convention
                double movementEndX = agentXInches + vectorScale * movementMagnitude * Math.cos(movementAngle);
                double movementEndY = agentYInches + vectorScale * movementMagnitude * Math.sin(movementAngle);

                fieldOverlay.setStroke("green");
                fieldOverlay.setStrokeWidth(2);
                fieldOverlay.strokeLine(agentXInches, agentYInches, movementEndX, movementEndY);
            }

            telemetry.addData("agent", "(%.2f, %.2f)", agentX, agentY);
            telemetry.addData("target", "(%.2f, %.2f)", targetX, targetY);
            telemetry.addData("Forward", forward);
            telemetry.addData("Strafe", strafe);
            telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
            telemetry.addData("DTT", DTT);
            telemetry.update();


        }
        yaiba.close();
    }


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
        if(gamepad1.a){
            targetX = 60.96f;
            targetY = 60.96f;
        }if(gamepad1.b){
            targetX = -60.96f;
            targetY = 60.96f;
        }if(gamepad1.x){
            targetX = -60.96f;
            targetY = -60.96f;
        }if(gamepad1.y){
            targetX = 60.96f;
            targetY = -60.96f;
        }
    }
}

