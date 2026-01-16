package org.firstinspires.ftc.teamcode.auto;

import static org.firstinspires.ftc.teamcode.components.Constants.actionsIndex;
import static org.firstinspires.ftc.teamcode.components.Constants.distanceTolerance;
import static org.firstinspires.ftc.teamcode.components.Constants.firstIntakePrepX;
import static org.firstinspires.ftc.teamcode.components.Constants.firstIntakePrepY;
import static org.firstinspires.ftc.teamcode.components.Constants.firstIntakeX;
import static org.firstinspires.ftc.teamcode.components.Constants.firstIntakeY;
import static org.firstinspires.ftc.teamcode.components.Constants.humanPlayerPrepX;
import static org.firstinspires.ftc.teamcode.components.Constants.humanPlayerPrepY;
import static org.firstinspires.ftc.teamcode.components.Constants.humanPlayerX;
import static org.firstinspires.ftc.teamcode.components.Constants.humanPlayerY;
import static org.firstinspires.ftc.teamcode.components.Constants.initHeading;
import static org.firstinspires.ftc.teamcode.components.Constants.positionRotation;
import static org.firstinspires.ftc.teamcode.components.Constants.reverseMultForward;
import static org.firstinspires.ftc.teamcode.components.Constants.reverseMultStrafe;
import static org.firstinspires.ftc.teamcode.components.Constants.secondIntakePrepX;
import static org.firstinspires.ftc.teamcode.components.Constants.secondIntakePrepY;
import static org.firstinspires.ftc.teamcode.components.Constants.secondIntakeX;
import static org.firstinspires.ftc.teamcode.components.Constants.secondIntakeY;
import static org.firstinspires.ftc.teamcode.components.Constants.shootTargetX;
import static org.firstinspires.ftc.teamcode.components.Constants.shootTargetY;
import static org.firstinspires.ftc.teamcode.components.Constants.startX;
import static org.firstinspires.ftc.teamcode.components.Constants.startY;
import static org.firstinspires.ftc.teamcode.components.Constants.targetId;
import static org.firstinspires.ftc.teamcode.components.Constants.thirdIntakePrepX;
import static org.firstinspires.ftc.teamcode.components.Constants.thirdIntakePrepY;
import static org.firstinspires.ftc.teamcode.components.Constants.thirdIntakeX;
import static org.firstinspires.ftc.teamcode.components.Constants.thirdIntakeY;

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
import org.firstinspires.ftc.teamcode.components.Constants;

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
        driveToShoot,
        shoot,
        firstIntakePrep,
        firstIntake,
        secondIntakePrep,
        secondIntake,
        thirdIntakePrep,
        thirdIntake,
        humanPlayerPrep,
        humanPlayer
    }

    public float agentX;
    public float agentY;
    private currentState currentState;
    private Constants constants;

    int currentPosition = 0;

    TurretSpin turretSpin;
    Turret turret;
    Color color;
    spindexAutoSort.targetMotif target;
    public float desiredHeading = 90;
    private float clamp(float v, float lo, float hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public int[] currentLayout = new int[]{-1, 1, 1};

    boolean sorted = false;

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
        if(state == currentState.driveToShoot){
            targetX = shootTargetX;
            targetY = shootTargetY;
            desiredHeading = 90;
            if((getAgentX() < shootTargetX + DISTANCE_TOLERANCE && getAgentX() > shootTargetX - DISTANCE_TOLERANCE) && getAgentY() < shootTargetY + DISTANCE_TOLERANCE && getAgentY() > shootTargetY - DISTANCE_TOLERANCE){
                state = currentState.shoot;
            }
        }
        if(state == currentState.shoot && !hasShot){
            hasShot = true;
            Shoot();
        }

        if(currentLayout[0] == 0 && currentLayout[1] == 0 && currentLayout[2] == 0){
            hasShot = false;
            if(shootCount == 0){
                state = currentState.firstIntakePrep;
                shootCount = 1;
            }
            else if(shootCount == 1){
                state = currentState.secondIntakePrep;
                shootCount = 2;
            }
            else if(shootCount == 2){
                state = currentState.thirdIntakePrep;
                shootCount = 3;
            }else{
                //state = currentState.humanPlayerPrep;
            }
        }

        if(state == currentState.firstIntakePrep) {
            targetX = firstIntakePrepX;
            targetY = firstIntakePrepY;
            if ((getAgentX() < firstIntakePrepX + DISTANCE_TOLERANCE && getAgentX() > firstIntakePrepX - DISTANCE_TOLERANCE) && getAgentY() < firstIntakePrepY + DISTANCE_TOLERANCE && getAgentY() > firstIntakePrepY - DISTANCE_TOLERANCE) {
                state = currentState.firstIntake;
            }
        }

        if(state == currentState.firstIntake){
            targetX = firstIntakeX;
            targetY = firstIntakeY;
            if((getAgentX() < firstIntakeX + DISTANCE_TOLERANCE && getAgentX() > firstIntakeX - DISTANCE_TOLERANCE) && getAgentY() < firstIntakeY + DISTANCE_TOLERANCE && getAgentY() > firstIntakeY - DISTANCE_TOLERANCE){
                state = currentState.driveToShoot;
            }
        }

        if(state == currentState.secondIntakePrep) {
            targetX = secondIntakePrepX;
            targetY = secondIntakePrepY;
            if ((getAgentX() < secondIntakePrepX + DISTANCE_TOLERANCE && getAgentX() > secondIntakePrepX - DISTANCE_TOLERANCE) && getAgentY() < secondIntakePrepY + DISTANCE_TOLERANCE && getAgentY() > secondIntakePrepY - DISTANCE_TOLERANCE) {
                state = currentState.secondIntake;
            }
        }

        if(state == currentState.secondIntake) {
            targetX = secondIntakeX;
            targetY = secondIntakeY;
            if ((getAgentX() < secondIntakeX + DISTANCE_TOLERANCE && getAgentX() > secondIntakeX - DISTANCE_TOLERANCE) && getAgentY() < secondIntakeY + DISTANCE_TOLERANCE && getAgentY() > secondIntakeY - DISTANCE_TOLERANCE) {
                state = currentState.driveToShoot;
            }
        }

        if(state == currentState.thirdIntakePrep) {
            targetX = thirdIntakePrepX;
            targetY = thirdIntakePrepY;
            if ((getAgentX() < thirdIntakePrepX + DISTANCE_TOLERANCE && getAgentX() > thirdIntakePrepX - DISTANCE_TOLERANCE) && getAgentY() < thirdIntakePrepY + DISTANCE_TOLERANCE && getAgentY() > thirdIntakePrepY - DISTANCE_TOLERANCE) {
                state = currentState.firstIntake;
            }
        }

        if(state == currentState.thirdIntake) {
            targetX = thirdIntakeX;
            targetY = thirdIntakeY;

            if ((getAgentX() < thirdIntakeX + DISTANCE_TOLERANCE && getAgentX() > thirdIntakeX - DISTANCE_TOLERANCE) && getAgentY() < thirdIntakeY + DISTANCE_TOLERANCE && getAgentY() > thirdIntakeY - DISTANCE_TOLERANCE) {
                state = currentState.driveToShoot;
            }
        }

//        if(state == currentState.humanPlayerPrep) {
//            targetX = humanPlayerPrepX;
//            targetY = humanPlayerPrepY;
        //    desiredHeading = 180;
//            if ((getAgentX() < humanPlayerPrepX + DISTANCE_TOLERANCE && getAgentX() > humanPlayerPrepX - DISTANCE_TOLERANCE) && getAgentY() < humanPlayerPrepY + DISTANCE_TOLERANCE && getAgentY() > humanPlayerPrepY - DISTANCE_TOLERANCE) {
//                state = currentState.humanPlayer;
//            }
//        }
//
//        if(state == currentState.humanPlayer) {
//            targetX = humanPlayerX;
//            targetY = humanPlayerY;
//            if ((getAgentX() < humanPlayerX + DISTANCE_TOLERANCE && getAgentX() > humanPlayerX - DISTANCE_TOLERANCE) && getAgentY() < humanPlayerY + DISTANCE_TOLERANCE && getAgentY() > humanPlayerY - DISTANCE_TOLERANCE) {
//                state = currentState.driveToShoot;
//            }
//        }
        currentState = state;
    }

    private void Shoot(){
        sorted = false;
        spindex.startShootConsecutive();
        currentPosition = (currentPosition + 2 ) % 3;
        currentLayout[currentPosition] = 0;
    }

    private float imuCorrection(double currentHeading) {
        // Ensure your desiredHeading is also in the 0-360 range (e.g., 90.0f)
        float offset = -1 * ((float) AngleUnit.normalizeDegrees(desiredHeading - currentHeading));


        telemetry.addData("offset", offset);
        if (Math.abs(offset) < 1f) return 0;
        return Math.copySign(Math.abs(offset) < 20f ? Constants.imuKp / 2 : Constants.imuKp, offset);
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

        currentState = currentState.driveToShoot;

        currentAmmo = new int[]{0, 0, 0};

        constants = new Constants();


    }

    @Override
    public void loop() {
        odo.update();
        Pose2D currentPose = odo.getPosition();

        turret.setPower(Constants.TURRET1);

        agentX = getAgentY();
        agentY = getAgentX();

        stateMachine(currentState);

        //NormalizedRGBA colors = color.getNormalizedColors();

        DTT = (float) Math.hypot(targetX - agentX, targetY - agentY);

        actions = yaiba.runDeterministic(agentX, agentY, targetX, targetY);

        strafe = actions[(int) actionsIndex];
        forward = actions[(int) (1 - actionsIndex)];


        if(Math.hypot(strafe, forward) < 0.10 && DTT > DISTANCE_TOLERANCE){
            strafe = (float) ((targetX - agentX)/(DISTANCE_TOLERANCE * Constants.autoFinalStageMultiplier));
            forward = (float) (targetY - agentY/(DISTANCE_TOLERANCE * Constants.autoFinalStageMultiplier));
        }


        double rawDeg = Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES));
        double positiveHeading = (rawDeg % 360 + 360) % 360;

        float powerMultipler = 3f;

        if (DTT > DISTANCE_TOLERANCE || currentState == currentState.firstIntakePrep || currentState  == currentState.secondIntakePrep || currentState == currentState.thirdIntakePrep || currentState == currentState.humanPlayerPrep) {
            drive.setPower(reverseMultForward * forward, reverseMultStrafe * strafe, imuCorrection(positiveHeading));
        }else{
            drive.setPower(0,0,0);
        }

        if(currentState == currentState.firstIntakePrep || currentState == currentState.secondIntakePrep || currentState == currentState.thirdIntakePrep){
            intake.setPower(1);
            //teleopFuncs.intakeCheck();
        }

        spindex.shootConsecutive();


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
                    autoSort.sortNShoot(currentLayout, target);
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
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("motors", "FL=%.2f FR=%.2f BL=%.2f BR=%.2f", fl, fr, bl, br);
        telemetry.addData("DTT", DTT);
        telemetry.addData("shootCount", shootCount);
        telemetry.addData("agentX", agentX);
        telemetry.addData("agentY", agentY);
        telemetry.addData("current Heading", odo.getHeading(AngleUnit.DEGREES));
        telemetry.addData("IMU Correction", imuCorrection(positiveHeading));
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

        if (result.getFiducialResults().get(0).getFiducialId() != targetId) {
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
