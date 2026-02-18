package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import android.content.res.AssetManager;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.automation.SpindexAutoSort;
import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.SpindexPID;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretAutoAimODO;
import org.firstinspires.ftc.teamcode.components.TurretRTP;
import org.firstinspires.ftc.teamcode.components.TurretRegression;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.components.Util;
import org.firstinspires.ftc.teamcode.yaiba.BODYONNX;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import ai.onnxruntime.OrtException;

import org.firstinspires.ftc.teamcode.components.AutoBlueConstants;

// TODO: intake further
@TeleOp
public class YAIBA_BLUEFRONT extends OpMode {

    private BODYONNX model;

    private GoBildaPinpointDriver odo;

    private double MODEL_POS_SCALE = AutoBlueConstants.MODEL_POS_SCALE;
    private static final double TARGET_X_M = 0.33;
    private static final double TARGET_Y_M = 0.15;

    private double robotX = 0, robotY = 0; // meters (for telemetry)
    private double targetX = TARGET_X_M;
    private double targetY = TARGET_Y_M;

    Drive drive;

    public Pose2D startPose;

    public float targetAngle = 0;
    public float stageX;
    public float stageY;
    public int stageActive;
    public float currentHeading;
    boolean notStarted;

    double scaled = 125;
    public boolean intakeCheckEnabled = false;
    AssetManager assetManager;

    double oldTime = 0;

    public ElapsedTime autoSortTimer = new ElapsedTime();
    public ElapsedTime intakePauseTimer = new ElapsedTime();
    public ElapsedTime timer = new ElapsedTime(); // ADDED (ElapsedTime-based shooting delay)
    boolean autoSortTimerStarted = false;


    public enum autoStage{
        firstShootDrive,
        firstShot,
        firstPickupSetup,
        firstPickup,
        shootDrive,
        shoot,
        gatePushSetup,
        gatePush,
        gatePickup,
        secondPickupSetup,
        secondPickup,
        thirdPickupSetup,
        thirdPickup,
        finish;
    }

    public autoStage currentStage;

    public float DTT;

    Spindex spindex;

    Intake intake;

    private int shootCnt = 0;

    String detectedColor;

    Color color;

    Turret turret;
    Hood hood;
    TurretSpin turretSpin;
    SpindexAutoSort autoSort;

    SpindexPID pid;
    TurretAutoAimODO aim;
    int currentPosition = 0;

    public boolean sorted = false;
    float intakeCount;

    int fortelemetry;
    int fortelemetry2;
    public int[] currentLayout = new int[]{0, 0, 0};

    private boolean startShoot = false;

    boolean startedTimer = false;

    public ElapsedTime gateTimer = new ElapsedTime();

    boolean arrived = false;

    public ElapsedTime intakeTimer = new ElapsedTime();

    boolean prevShootingBoolean = false;

    boolean hasPushed = false;

    boolean motifCheck = false;


    private float[] buildObservations() {
        float[] obs = new float[9];

        // 1. Target - Robot (Matches Unity: targetPos - transform.localPosition)
        // 2. Axis Swap: FTC X is Forward (Unity Z), FTC Y is Lateral (Unity X)
        // 3. Handedness: FTC +Y is Left, Unity +X is Right. So negate the Lateral axis.
        double relLateral = -(targetY - robotY);
        double relForward = (targetX - robotX);

        DTT = (float) Math.hypot(relLateral, relForward);

        // REMOVE THE / 10f. Your C# code does not use it.
        obs[0] = (float) relLateral; // AI Slot 0: Lateral (Unity X)
        obs[1] = (float) relForward; // AI Slot 1: Forward (Unity Z)

        // 4. Heading: Both must be negated to convert FTC CCW to Unity CW
        obs[2] = (float)Math.sin(-currentHeading);
        obs[3] = (float)Math.cos(-currentHeading);

        // You missed the negation on targetAngle in your previous code!
        obs[4] = (float)Math.sin(-targetAngle);
        obs[5] = (float)Math.cos(-targetAngle);

        obs[6] = stageActive; // stageActive
        obs[7] = stageX;
        obs[8] = stageY;

        return obs;
    }

    private void stateMachine(){
        //gate push angle is 120 degrees
        // if(AutoConstants.currentAuto == AutoConstants.autoMode.blueFront) {
        switch(currentStage){
            case firstShootDrive:
                targetX = AutoBlueConstants.shootX;
                targetY = AutoBlueConstants.shootY;
                targetAngle = -1.578f;
                AutoBlueConstants.driveForwardMult = 1;
                AutoBlueConstants.driveStrafeMult = -1;
// TODO: Change Goal
                buildObservations();
                if(DTT < 0.05){
                    startShoot = true;
                    scaled = AutoBlueConstants.shootScaled1;
                    currentStage = autoStage.firstShot;
                    timer.reset(); // ADDED: start delay timer when entering shoot stage
                }
                break;
            case firstShot:
                intake.setPower(0);
                if(timer.milliseconds() > AutoBlueConstants.beforeShootDelayMS) {
                    if (startShoot && turret.atTarget()) {
                        pid.setTargetStep(-3);
                        startShoot = false;
                    }
                    if (pid.isAtTarget() && !startShoot) {
                        intakeCheckEnabled = true;
                        shootCnt++;
                        currentStage = autoStage.firstPickupSetup;
                        motifCheck = true;
                        timer.reset();
                    }
                }
                break;

            case shoot:
                arrived = false;
                intake.setPower(0);
                if(timer.milliseconds() > AutoBlueConstants.beforeShootDelayMS) {
                    if (startShoot) {
                        pid.startShootConsecutive();
                        startShoot = false;
                    }
                    if (!pid.shooting) {
                        intakeCheckEnabled = true;
                        shootCnt++;
                        if (shootCnt == 2) currentStage = autoStage.gatePushSetup;
                        if (shootCnt == 3) currentStage = autoStage.secondPickupSetup;
                        if (shootCnt == 4) currentStage = autoStage.thirdPickupSetup;
                        if (shootCnt == 5) currentStage = autoStage.finish;
                        timer.reset();
                    }
                }
                break;

            case firstPickupSetup:
                targetX = AutoBlueConstants.intake1PrepX;
                targetY = AutoBlueConstants.intakePrepY;
                targetAngle = -1.578f;
// TODO: Change Goal
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.firstPickup;
                }
                break;

            case firstPickup:
                intake.setPower(1);
                targetX = AutoBlueConstants.intake1PrepX;
                targetY = AutoBlueConstants.intakeY;
                AutoBlueConstants.driveForwardMult = 0.75f;
                AutoBlueConstants.driveStrafeMult = -0.75f;
                buildObservations();
                intakeCheckEnabled = true;
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoBlueConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                        //intake.togglePower(intake.intakePower);
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    intake.setPower(-0.25f);
                    currentStage = autoStage.shootDrive;
                }
                break;

            case shootDrive:
                targetX = AutoBlueConstants.shootX;
                targetY = AutoBlueConstants.shootY;
                targetAngle = -1.578f;
                AutoBlueConstants.driveForwardMult = 1f;
                AutoBlueConstants.driveStrafeMult = -1f;
// TODO: Change Goal
                buildObservations();
                if(DTT< 0.05){
                    startShoot = true;
                    scaled = AutoBlueConstants.shootScaled2;
                    currentStage = autoStage.shoot;
                    // NOTE: red version does NOT reset timer here, so we keep behavior consistent
                }
                break;

            case gatePushSetup:
                targetX = AutoBlueConstants.gatePushPrepX;
                targetY = AutoBlueConstants.gatePushPrepY;
                //targetAngle = -2.0944f;
                buildObservations();
                if(DTT< 0.05){
                    currentStage = autoStage.gatePush;
                }
                break;

            case gatePush:
                targetX = AutoBlueConstants.gatePushPrepX;
                targetY = AutoBlueConstants.gatePushY;
                AutoBlueConstants.driveForwardMult = 0.75f;
                AutoBlueConstants.driveStrafeMult = -0.75f;
                buildObservations();
                intake.setPower(1);
                if(DTT < 0.05){
                    if(!hasPushed){
                        hasPushed = true;
                        gateTimer.reset();
                    }else if(gateTimer.seconds() < AutoBlueConstants.pushTime){
                        currentStage = autoStage.gatePickup;
                    }
                }
                break;
            case gatePickup:
                intake.setPower(1);
                targetX = AutoBlueConstants.gatePickupX;
                targetY = AutoBlueConstants.gatePickupY;
                targetAngle = (float) AutoBlueConstants.gateHeading;
                buildObservations();
                if(DTT < 0.05){
                    //intake.setPower(1);
                    if(!startedTimer){
                        gateTimer.reset();
                        startedTimer = true;
                    }else if(gateTimer.seconds() > AutoBlueConstants.gateTime){
                        intake.setPower(-0.25f);
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    intake.setPower(-0.25f);
                    currentStage = autoStage.shootDrive;
                }
                break;
            case secondPickupSetup:
                targetX = AutoBlueConstants.intake2PrepX;
                targetY = AutoBlueConstants.intakePrepY;
                AutoBlueConstants.driveForwardMult = 1f;
                AutoBlueConstants.driveStrafeMult = -1f;
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.secondPickup;
                }
                break;

            case secondPickup:
                targetX = AutoBlueConstants.intake2PrepX;
                targetY = AutoBlueConstants.intakeY;
                AutoBlueConstants.driveForwardMult = 0.75f;
                AutoBlueConstants.driveStrafeMult = -0.75f;
                intake.setPower(1);
                buildObservations();
                //intake issue
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoBlueConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    intake.setPower(-0.25f);
                    currentStage = autoStage.shootDrive;
                }
                break;

            case thirdPickupSetup:
                targetX = AutoBlueConstants.intake3PrepX;
                targetY = AutoBlueConstants.intakePrepY;
                AutoBlueConstants.driveForwardMult = 1f;
                AutoBlueConstants.driveStrafeMult = -1f;
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.thirdPickup;
                }
                break;

            case thirdPickup:
                targetX = AutoBlueConstants.intake3PrepX;
                targetY = AutoBlueConstants.intakeY;
                AutoBlueConstants.driveForwardMult = 0.75f;
                AutoBlueConstants.driveStrafeMult = -0.75f;
                buildObservations();
                intake.setPower(1);
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoBlueConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    intake.setPower(-0.25f);
                    currentStage = autoStage.shootDrive;
                }
                break;

            case finish:
                targetX = 0.5f;
                targetY = 0f;
                break;
        }
        //}
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap);
        color = new Color(hardwareMap);
        spindex = new Spindex(hardwareMap);
        intake = new Intake(hardwareMap);

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        autoSort = new SpindexAutoSort(hardwareMap, telemetry);

        pid = new SpindexPID(hardwareMap);

        aim = new TurretAutoAimODO(hardwareMap, -0.944, -0.66);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(12, -17.5, DistanceUnit.CM);



        while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY && !Thread.currentThread().isInterrupted()) {
            odo.update();
            telemetry.addData("status", odo.getDeviceStatus());
            telemetry.addData("status2",Thread.currentThread().isInterrupted() );
            telemetry.update();
        }

        odo.setPosition(new Pose2D(DistanceUnit.CM, -0.944 / MODEL_POS_SCALE, -0.66 / MODEL_POS_SCALE, AngleUnit.RADIANS, -1.578f));
        currentStage = autoStage.firstShootDrive;
        try {
            // Load AI model
            model = new BODYONNX(hardwareMap.appContext.getAssets());
            telemetry.addData("Status", "Model loaded successfully");
            telemetry.addData("initial position", odo.getPosition().getX(DistanceUnit.CM));
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load model: " + e.getMessage());
            telemetry.update();
            return;
        }


        android.content.Context context = hardwareMap.appContext;
        assetManager = context.getAssets();



    }

    @Override
    public void loop() {

        odo.update();
        turret.loop();

        Pose2D currentPose = odo.getPosition();
        robotX = (currentPose.getX(DistanceUnit.CM) * MODEL_POS_SCALE);
        robotY = (currentPose.getY(DistanceUnit.CM) * MODEL_POS_SCALE);
        currentHeading = (float) currentPose.getHeading(AngleUnit.DEGREES);

        // Build observations
        float[] observations = buildObservations();

        updateColor();
        stateMachine();

        // CALCULATE CHECKSUM - should change every frame if observations change
        float obsChecksum = 0;
        for (float obs : observations) {
            obsChecksum += obs;
        }

        // Get AI predictions
        boolean inferenceSuccess = false;
        long inferenceTime = 0;
        long startTime = System.nanoTime();
        float[] actions = null;
        try {
            actions = model.predict(observations, assetManager);
        } catch (OrtException e) {
            throw new RuntimeException(e);
        }
        inferenceTime = (System.nanoTime() - startTime) / 1_000_000;
        inferenceSuccess = true;

        // Calculate action checksum
        float actionChecksum = actions[0] + actions[1] + actions[2];

        float strafe = -actions[0];
        float forward = actions[1];
        float rotation = actions[2];

        float denominator = Math.max(Math.abs(forward) + Math.abs(strafe) + Math.abs(rotation), 1.0f);

        forward /= denominator;
        strafe /= denominator;
        rotation /= denominator;

        drive.setPower(forward * AutoBlueConstants.driveForwardMult, strafe * AutoBlueConstants.driveStrafeMult, rotation * AutoBlueConstants.driveRotationMult);
        pid.update();

        aim.runToAim(telemetry);

        pid.shootConsecutive(color);
        //motifCheck();


        updateColor();
        if(intake.getPower() != 0 && intakeCheckEnabled){
            intakeCheck();
        }
        if(intake.paused) intake.pause(Constants.intakeReverseTime, intakePauseTimer, Intake.intakePower);



        hood.setPosition(0.675);
        turret.setTargetRPM(135);
        telemetry.addData("target hood pos", hood.getPosition());
        telemetry.addData("target turret rpm", 140);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Convert meters to inches for FTC Dashboard (uses official field frame in inches)
        double robotXInches = (robotX ) * AutoBlueConstants.telemetryScale;
        double robotYInches = (robotY ) * AutoBlueConstants.telemetryScale;
        double targetXInches = targetX * AutoBlueConstants.telemetryScale;
        double targetYInches = targetY * AutoBlueConstants.telemetryScale;

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
        fieldOverlay.strokeCircle(robotXInches, robotYInches, 9);
        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(robotXInches, robotYInches, 9);

        // Draw direction line on robot
        double lineLength = 9;
        double lineEndX = robotXInches + lineLength * Math.cos(odo.getPosition().getHeading(AngleUnit.DEGREES));
        double lineEndY = robotYInches + lineLength * Math.sin(odo.getPosition().getHeading(AngleUnit.DEGREES));
        fieldOverlay.setStroke("white");
        fieldOverlay.setStrokeWidth(2);
        fieldOverlay.strokeLine(robotXInches, robotYInches, lineEndX, lineEndY);

        // Draw desired movement vector (green line showing forward/strafe direction)
        double movementMagnitude = Math.sqrt(forward * forward + strafe * strafe);
        if (movementMagnitude > 0.01) {
            // Scale the movement vector for visibility (20 inches at full power)
            double vectorScale = 20.0;
            double headingRad = currentPose.getHeading(AngleUnit.DEGREES); // actually in radians
            double fieldDx = (forward * Math.cos(headingRad)) - (strafe * Math.sin(headingRad));
            double fieldDy = (forward * Math.sin(headingRad)) + (strafe * Math.cos(headingRad));
            double movementEndX = robotXInches + vectorScale * fieldDx;
            double movementEndY = robotYInches + vectorScale * fieldDy;

            fieldOverlay.setStroke("green");
            fieldOverlay.setStrokeWidth(2);
            fieldOverlay.strokeLine(robotXInches, robotYInches, movementEndX, movementEndY);
        }

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Telemetry
        telemetry.addData("=== ODOMETRY ===", " ");
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency());
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        telemetry.addData("REV Hub Frequency: ", frequency);
        telemetry.addData("Heading Scalar", odo.getYawScalar());
        telemetry.addData("=== DIAGNOSTICS ===", "");
        telemetry.addData("Inference Success?", inferenceSuccess);
        telemetry.addData("Inference Time (ms)", inferenceTime);
        telemetry.addData("Obs Checksum", "%.4f", obsChecksum);
        telemetry.addData("Action Checksum", "%.4f", actionChecksum);

        telemetry.addData("=== Position ===", "");
        telemetry.addData("Position", "%.2f, %.2f", robotX, robotY);
        telemetry.addData("Target", "%.2f, %.2f", targetX, targetY);
        telemetry.addData("Rotation", "%.2f deg", Math.toDegrees(odo.getPosition().getHeading(AngleUnit.DEGREES)));

        telemetry.addData("=== Raw Observations ===", "");
        for (int i = 0; i < observations.length; i++) {
            telemetry.addData("obs[" + i + "]", "%.3f", observations[i]);
        }

        telemetry.addData("=== Actions ===", "");
        telemetry.addData("Actions", "F:%.3f S:%.3f R:%.3f", forward, strafe, rotation);

        telemetry.addData("=== Pathing ===", "");
        telemetry.addData("Current Stage", currentStage);
        telemetry.addData("Current Auto", AutoBlueConstants.currentAuto);
        telemetry.addData("DTT", DTT);
        telemetry.addData("Shooting", pid.shooting);

        telemetry.addData("=== COLOR ===", "");
        telemetry.addData("Color", color.getColor());


        telemetry.update();
    }

    public void updateColor(){
        if(!pid.shooting && prevShootingBoolean){
            // Falling edge -> it just finished shooting
            currentLayout = new int[]{0, 0, 0};
        }
        prevShootingBoolean = pid.shooting;

        if(!pid.isAtTarget()){
            return;
        }
        else if(color.getColor() == "GREEN") {
            currentLayout[currentPosition] = 1;
        }
        else if(color.getColor() == "PURPLE") {
            currentLayout[currentPosition] = -1;
        }
        else if(color.getColor() == "NONE"){
            currentLayout[currentPosition] = 0;
        }

        detectedColor = color.getColor();
    }

    public void intakeCheck() {
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

        if (!pid.isAtTarget()){
            return;
        }

        if ((detectedColor == "GREEN" || detectedColor == "PURPLE") && pid.isAtTarget()){
            if(autoSortTimerStarted && autoSortTimer.milliseconds() >= Constants.autoSortDelayMs){
                if(purCount + grnCount <= 2){
                    sorted = false;
                    intake.paused = true;
                    intakePauseTimer.reset();
                    autoSortTimer.reset();
                    autoSortTimerStarted = false;
                    intakeCount++;
                    pid.setTargetStep(1);
                    currentPosition = (currentPosition + 1) % 3;
                }
                else{
                    fortelemetry = currentPosition;
                    int turns = autoSort.sortNShoot(currentLayout, turretSpin.detectedMotif, currentPosition);
                    pid.setTargetStep(turns);
                    telemetry.addData("auto sort", autoSort.sortNShoot(currentLayout, turretSpin.detectedMotif, currentPosition));
                    currentPosition = (currentPosition + turns) % 3;
                    fortelemetry2 = currentPosition;
                    sorted = true;
                    intake.togglePower(Intake.intakePower);
                }
            } else if(!autoSortTimerStarted){
                autoSortTimerStarted = true;
                autoSortTimer.reset();
            }
        }
    }

//    public void motifCheck(){
//        if(!motifCheck) return;
//
//        turretSpin.autoAim();
//
//        if(turretSpin.result != null || turretSpin.result.isValid()){
//            if(turretSpin.result.getFiducialResults().get(0).getFiducialId() >= 21 && turretSpin.result.getFiducialResults().get(0).getFiducialId() <= 23){
//                rtp.targetPosition = rtp.getSafeTarget(45);
//                motifCheck = false;
//            }
//        }else{
//            rtp.targetPosition = rtp.getSafeTarget(90);
//        }
//    }
}
