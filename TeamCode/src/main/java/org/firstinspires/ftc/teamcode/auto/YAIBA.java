package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.components.AutoConstants.*;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.sun.tools.javac.util.Context;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.automation.spindexAutoSort;
import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.teleop.teleop;
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
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.AutoConstants;

@Autonomous
public class YAIBA extends OpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    // TFLite wrapper
    private BODY yaiba;
    private GoBildaPinpointDriver odo;
    private Spindex spindex;

    private float[] actions;

    private float targetX = 0f;
    private float targetY = 0f;

    private double fl;
    private double fr;
    private double bl;
    private double br;
    private float forward;
    private float strafe;
    private float rot;
    private int cooldownCounter = 0;

    public static final float DISTANCE_TOLERANCE = distanceTolerance;
    private float DTT;

    private float shootCount = 0;

    private int[] currentAmmo = new int[3];

    private spindexAutoSort autoSort;

    String detectedMotif = "None Detected";

    boolean hasShot = false;

    String detectedColor;

    Intake intake;

    Drive drive;

    private enum currentState{
        shoot,
        firstIntake,
        secondIntake,
        thirdIntake,
    }

    public float agentX;
    public float agentY;
    public float targetStageX;
    public float targetStageY;
    private currentState currentState;
    private AutoConstants AutoConstants;

    int currentPosition = 0;

    TurretSpin turretSpin;
    Turret turret;
    Color color;
    spindexAutoSort.targetMotif target;
    public float desiredHeading = -1.5708f;
    private float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public int[] currentLayout = new int[]{0, 0, 0};

    boolean sorted = false;

    float stageActive;

    boolean processingBall = false;
    int intakeCount = 0;

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

    private void stateMachine(currentState state){
        if(state == currentState.shoot){
            targetX = 0;
            targetY = 0;
            stageActive = 0;
            if(DTT < 5f && !hasShot){
                hasShot = true;
                spindex.shootConsecutive(color);
            }
//            if()
        }
        currentState = state;
    }

    private void Shoot(){
        sorted = false;
        spindex.startShootConsecutive();
        currentPosition = (currentPosition + 2 ) % 3;
        currentLayout[currentPosition] = 0;
    }



    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpin = new TurretSpin(hardwareMap);
        turret = new Turret(hardwareMap);
        spindex = new Spindex(hardwareMap);
        color = new Color(hardwareMap);
        intake = new Intake(hardwareMap);

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

        drive = new Drive(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(16.5, -19, DistanceUnit.CM);
        Pose2D startPose = new Pose2D(DistanceUnit.CM, startX, startY, AngleUnit.DEGREES, initHeading);
        odo.setPosition(startPose);

        currentState = currentState.shoot;

        currentAmmo = new int[]{0, 0, 0};

        AutoConstants = new AutoConstants();


    }

    @Override
    public void loop() {
        odo.update();
        Pose2D currentPose = odo.getPosition();

        turret.setTargetRPM(Turret.targetRPM1);

        agentX = getAgentY();
        agentY = getAgentX();

        float relX = targetX - agentX;
        float relY = targetY - agentY;

        stateMachine(currentState);

        updateColor();

        //NormalizedRGBA colors = color.getNormalizedColors();

        DTT = (float) Math.hypot(targetX - agentX, targetY - agentY);

        actions = yaiba.runDeterministic(relX, relY, (float) Math.sin(currentPose.getHeading(AngleUnit.RADIANS)), (float) Math.sin(currentPose.getHeading(AngleUnit.RADIANS)), (float) Math.sin(desiredHeading), (float) Math.cos(desiredHeading), stageActive, targetStageX, targetStageY);

        strafe = actions[(int) actionsIndex];
        forward = actions[(int) (1 - actionsIndex)];
        rot = actions[3];

//
//        if(Math.hypot(strafe, forward) < 0.10 && DTT > DISTANCE_TOLERANCE){
//            strafe = (float) ((targetX - agentX)/(DISTANCE_TOLERANCE * AutoConstants.autoFinalStageMultiplier));
//            forward = (float) (targetY - agentY/(DISTANCE_TOLERANCE * AutoConstants.autoFinalStageMultiplier));
//        }


        double rawDeg = Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES));
        double positiveHeading = (rawDeg % 360 + 360) % 360;

        drive.setPower(forward, strafe, rot);

        spindex.shootConsecutive(color);


        if(intake.getPower() != 0){
            intakeCheck();

            int grnCount = 0;
            int purCount = 0;
            for(int i = 0; i <= 2; i ++){
                if(currentLayout[i] == 1){
                    grnCount++;
                }
                if(currentLayout[i] == -1){
                    purCount++;
                }
            }
            if(grnCount + purCount >= 3 && !sorted){
                if(!spindex.withinTarget()){
                    autoSort.sortNShoot(currentLayout, target, currentPosition);
                    sorted = true;
                }
            }
        }

        autoAim();

        // Create telemetry packet with field overlay
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        double heading = currentPose.getHeading  (AngleUnit.RADIANS);

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
        telemetry.addData("current Heading", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("activeStage", stageActive);
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rot", rot);
        telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
        telemetry.addData("DTT", DTT);
        telemetry.addData("shootCount", shootCount);
        telemetry.update();
    }

    public void updateColor(){
        if(!spindex.withinTarget()){
            return;
        }
        if(color.getColor() == "GREEN") {
            currentLayout[currentPosition] = 1;
        }
        if(color.getColor() == "PURPLE") {
            currentLayout[currentPosition] = -1;
        }
        if(color.getColor() == "NONE"){
            currentLayout[currentPosition] = 0;
        }

        detectedColor = color.getColor();
    }

    public void autoAim() {
        LLResult result = turretSpin.limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            turretSpin.spinRightCR(0);
            return;
        }

        if (result.getFiducialResults().get(0).getFiducialId() != Constants.targetId) {
            if (result.getFiducialResults().get(0).getFiducialId() == 21) {
                target = spindexAutoSort.targetMotif.GPP;
                detectedMotif = "GPP";
            } else if (result.getFiducialResults().get(0).getFiducialId() == 22) {
                target = spindexAutoSort.targetMotif.PGP;
                detectedMotif = "PGP";
            } else if (result.getFiducialResults().get(0).getFiducialId() == 23) {
                target = spindexAutoSort.targetMotif.PPG;
                detectedMotif = "PPG";
            }
            return;
        }

        double tx = result.getTx();
        // if tx is positive (target to the right) the turret needs to rotate right to center therefore -tx
// most motors rotate + pwr = turn right. so we shouldnt need to make it negative??
//using if no target then stop

        //computing how much error:
        double error = tx; //u want tx=0

//i feel like we should have a deadzone bc its continuous
        double deadband = 1.0; //degrees
        if (Math.abs(error) < deadband) {
            turretSpin.spinRightCR(0);
            return;
        }
    }

    public void intakeCheck() {
        int grnCount = 0;
        int purCount = 0;
        for (int i = 0; i <= 2; i++) {
            if (currentLayout[i] == 1) {
                grnCount++;
            }
            if (currentLayout[i] == -1) {
                purCount++;
            }
        }
        if ((grnCount + purCount) >= 3) {
            return;
        }


        if (!spindex.withinTarget()) {
            return;
        }
        if (detectedColor == "NONE") {
            processingBall = false;
        } else if ((detectedColor == "GREEN" || detectedColor == "PURPLE") && spindex.withinTarget() && !processingBall) {
            processingBall = true;
            spindex.spinTurns(1);
            currentPosition = (currentPosition + 1) % 3;
            intakeCount++;
        }
    }
}
