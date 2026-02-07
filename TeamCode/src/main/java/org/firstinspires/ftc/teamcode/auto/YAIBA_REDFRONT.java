package org.firstinspires.ftc.teamcode.auto;

import android.content.res.AssetManager;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.automation.SpindexAutoSort;
import org.firstinspires.ftc.teamcode.components.AutoConstants;
import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretRegression;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.components.Util;
import org.firstinspires.ftc.teamcode.yaiba.BODYONNX;

import java.util.Objects;

import ai.onnxruntime.OrtException;

@TeleOp
public class YAIBA_REDFRONT extends OpMode {

    private BODYONNX model;

    private GoBildaPinpointDriver odo;

    private double MODEL_POS_SCALE = AutoConstants.MODEL_POS_SCALE;
    private static final double TARGET_X_M = 0.33;
    private static final double TARGET_Y_M = 0.15;

    private double robotX = 0, robotY = 0; // meters (for telemetry)
    private double targetX = TARGET_X_M;
    private double targetY = TARGET_Y_M;

    double scaled = 125;

    Drive drive;

    public Pose2D startPose;

    public float targetAngle = 0;
    public float stageX;
    public float stageY;
    public float currentHeading;
    boolean notStarted;

    AssetManager assetManager;

    double oldTime = 0;

    private boolean startShoot;

    

    public enum autoStage{
        firstShootDrive,
        firstPickupSetup,
        firstPickup,
        shootDrive,
        shoot,
        gatePushSetup,
        gatePush,
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
    int currentPosition = 0;
    double distToAprilTag = 1;

    public ElapsedTime autoSortTimer = new ElapsedTime();
    public ElapsedTime intakePauseTimer = new ElapsedTime();
    boolean autoSortTimerStarted = false;

    public boolean sorted = false;
    float intakeCount;
    String detectedMotif = Constants.defaultMotif;
    double lastAutoAimPower = 0;

    int fortelemetry;
    int fortelemetry2;

    public int[] currentLayout = new int[]{0, 0, 0};


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

        obs[6] = 0.0f; // stageActive
        obs[7] = 0f;
        obs[8] = 0f;

        return obs;
    }

    private void stateMachine(){
        //gate push angle is 120 degrees
//        if(AutoConstants.currentAuto == AutoConstants.autoMode.blueFront) {
//            switch(currentStage){
//                case firstShootDrive:
//                    targetX = -0.33f;
//                    targetY = -0.33f;
//                    targetAngle = -1.578f;
//                    AutoConstants.driveForwardMult = 1;
//                    AutoConstants.driveStrafeMult = 1;
//                    if(DTT < 0.05){
//                        currentStage = autoStage.shoot;
//                    }
//                case shoot:
//                    spindex.startShootConsecutive();
//                    if(!spindex.shooting){
//                        shootCnt++;
//                        if(shootCnt == 1) currentStage = autoStage.firstPickupSetup;
//                        if(shootCnt == 2) currentStage = autoStage.gatePushSetup;
//                        if(shootCnt == 3) currentStage = autoStage.secondPickupSetup;
//                        if(shootCnt == 4) currentStage = autoStage.thirdPickupSetup;
//                        if(shootCnt == 5) currentStage = autoStage.finish;
//                    }
//                case firstPickupSetup:
//                    targetX = 0.15f;
//                    targetY = -0.33f;
//                    if(DTT < 0.05){
//                        currentStage = autoStage.firstPickup;
//                    }
//                case firstPickup:
//                    intake.togglePower(intake.intakePower);
//                    targetX = 0.15f;
//                    targetY = -0.75f;
//                    AutoConstants.driveForwardMult = 0.35f;
//                    AutoConstants.driveStrafeMult = 0.35f;
//                    //intakeCheck code (idk how we're gonna implement)
//                    if(DTT < 0.025){
//                        currentStage = autoStage.shootDrive;
//                    }
//                case shootDrive:
//                    targetX = 0;
//                    targetY = 0;
//                    AutoConstants.driveForwardMult = 1f;
//                    AutoConstants.driveStrafeMult = 1f;
//                    if(DTT< 0.05){
//                        currentStage = autoStage.shoot;
//                    }
//                case gatePushSetup:
//                    targetX = - 0.15f;
//                    targetY = -0.66f;
//                    targetAngle = -2.0944f;
//                    if(DTT< 0.05){
//                        currentStage = autoStage.gatePush;
//                    }
//                case gatePush:
//                    targetX = - 0.15f;
//                    targetY = -0.8f;
//                    AutoConstants.driveForwardMult = 0.35f;
//                    AutoConstants.driveStrafeMult = 0.35f;
//                    intake.togglePower(intake.intakePower);
//                    //same issue as ball pickup
//                    //if(ballCount == 3){
//                    //currentStage = autoStage.driveToShoot
//                case secondPickupSetup:
//                    targetX = 0.41f;
//                    targetY = -0.33f;
//                    AutoConstants.driveForwardMult = 1f;
//                    AutoConstants.driveStrafeMult = 1f;
//                    if(DTT < 0.05){
//                        currentStage = autoStage.secondPickup;
//                    }
//                case secondPickup:
//                    targetX = 0.41f;
//                    targetY = -0.80f;
//                    AutoConstants.driveForwardMult = 0.35f;
//                    AutoConstants.driveStrafeMult = 0.35f;
//                    intake.togglePower(intake.intakePower);
//                    //intake issue
//                    if(DTT < 0.025){
//                        currentStage = autoStage.shootDrive;
//                    }
//                case thirdPickupSetup:
//                    targetX =-0.15f;
//                    targetY = -0.33f;
//                    AutoConstants.driveForwardMult = 0.8f;
//                    AutoConstants.driveStrafeMult = 0.8f;
//                    if(DTT < 0.05){
//                        currentStage = autoStage.thirdPickup;
//                    }
//                case thirdPickup:
//                    targetX = -0.15f;
//                    targetY = -0.8f;
//                    AutoConstants.driveForwardMult = 0.35f;
//                    AutoConstants.driveStrafeMult = 0.35f;
//                    if(DTT < 0.025){
//                        currentStage = autoStage.shootDrive;
//                    }
//                case finish:
//                    targetX = -0.33f;
//                    targetY = 0f;
//            }
//        }
        //if(AutoConstants.currentAuto == AutoConstants.autoMode.redFront) {
            switch(currentStage){
                case firstShootDrive:
                    targetX = -0.15;
                    targetY = 0;
                    targetAngle = 1.578f;

                    buildObservations();
                    AutoConstants.driveForwardMult = 1;
                    AutoConstants.driveStrafeMult = -1;
                    if(DTT < 0.05){
                        currentStage = autoStage.shoot;
                        startShoot = true;
                    }
                    break;
                case shoot:
                    if(startShoot){
                        spindex.startShootConsecutive();
                        startShoot = false;
                    }
                    if(!spindex.shooting){
                        shootCnt++;
                        if(shootCnt == 1) currentStage = autoStage.firstPickupSetup;
                        if(shootCnt == 2) currentStage = autoStage.gatePushSetup;
                        if(shootCnt == 3) currentStage = autoStage.secondPickupSetup;
                        if(shootCnt == 4) currentStage = autoStage.thirdPickupSetup;
                        if(shootCnt == 5) currentStage = autoStage.finish;
                    }
                    break;
                case firstPickupSetup:
                    targetX = 0.15f;
                    targetY = 0.33f;
                    buildObservations();
                    if(DTT < 0.05){
                        currentStage = autoStage.firstPickup;
                    }
                    break;
                case firstPickup:
                    intake.togglePower(intake.intakePower);
                    targetX = 0.15f;
                    targetY = 0.75f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 0.35f;
                    AutoConstants.driveStrafeMult = -0.35f;
                    //intakeCheck code (idk how we're gonna implement)
                    if(DTT < 0.05){
                        currentStage = autoStage.shootDrive;
                    }
                    break;
                case shootDrive:
                    targetX = 0;
                    targetY = 0;
                    buildObservations();
                    AutoConstants.driveForwardMult = 1f;
                    AutoConstants.driveStrafeMult = -1f;
                    if(DTT< 0.05){
                        currentStage = autoStage.shoot;
                        startShoot = true;
                    }
                    break;
                case gatePushSetup:
                    targetX = -0.66f;
                    targetY = -0.15f;
                    buildObservations();
                    targetAngle = 2.0944f;
                    if(DTT< 0.05){
                        currentStage = autoStage.gatePush;
                    }
                    break;
                case gatePush:
                    targetX =  0.15f;
                    targetY = 0.66f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 0.35f;
                    AutoConstants.driveStrafeMult = -0.35f;
                    intake.togglePower(intake.intakePower);
                    break;
                    //same issue as ball pickup
                    //if(ballCount == 3){
                    //currentStage = autoStage.driveToShoot
                case secondPickupSetup:
                    targetX = 0.4f;
                    targetY = 0.33f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 1f;
                    AutoConstants.driveStrafeMult = -1f;
                    if(DTT < 0.05){
                        currentStage = autoStage.secondPickup;
                    }
                    break;
                case secondPickup:
                    targetX = 0.4f;
                    targetY = 0.75f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 0.35f;
                    AutoConstants.driveStrafeMult = -0.35f;
                    intake.togglePower(intake.intakePower);
                    //intake issue
                    if(DTT < 0.05){
                        currentStage = autoStage.shootDrive;
                    }
                    break;
                case thirdPickupSetup:
                    targetX = -0.15f;
                    targetY = 0.33f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 0.8f;
                    AutoConstants.driveStrafeMult = -0.8f;
                    if(DTT < 0.05){
                        currentStage = autoStage.thirdPickup;
                    }
                    break;
                case thirdPickup:
                    targetX = -0.15f;
                    targetY = 0.75f;
                    buildObservations();
                    AutoConstants.driveForwardMult = 0.35f;
                    AutoConstants.driveStrafeMult = -0.35f;
                    if(DTT < 0.05){
                        currentStage = autoStage.shootDrive;
                    }
                    break;
                case finish:
                    targetX = 0.33;
                    targetY = 0f;
                    buildObservations();
            }
       // }
    }


    //update the odo pods

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        drive = new Drive(hardwareMap);
        autoSort = new SpindexAutoSort(hardwareMap, telemetry);

        color = new Color(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.setOffsets(12, -17.5, DistanceUnit.CM);

//        if(AutoConstants.allianceColor == "Red"){
//            if(AutoConstants.startingPosition == "Front"){
//                AutoConstants.currentAuto = AutoConstants.autoMode.redFront;
//            }else{
//                AutoConstants.currentAuto = AutoConstants.autoMode.redBack;
//            }
//        }else{
//            if(AutoConstants.startingPosition == "Front"){
//                AutoConstants.currentAuto = AutoConstants.autoMode.blueFront;
//            }else{
//                AutoConstants.currentAuto = AutoConstants.autoMode.blueBack;
//            }
//        }
        //startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, -1.578);

//        double startX = 0;
//        double startY = 0;
//        double startHeading = 0;
//        if(AutoConstants.currentAuto == null){
//            if(Objects.equals(AutoConstants.allianceColor, "Red")){
//                if(Objects.equals(AutoConstants.startingPosition, "Front")){
//                    startX = -0.944;
//                    startY = 0.66;
//                    startHeading = 1.578;
//                }else{
//                    //nothing for now
//                }
//            }else{
//                if(Objects.equals(AutoConstants.startingPosition, "Front")){
//                    startX = -0.944;
//                    startY = -0.66;
//                    startHeading = -1.578;
//                }else{
//                    //nothing for now
//                }
//            }
//        }else{
//            if (AutoConstants.currentAuto == AutoConstants.autoMode.blueFront) {
//                startX = -0.944;
//                startY = -0.66;
//                startHeading = -1.578;
//            } else if (AutoConstants.currentAuto == AutoConstants.autoMode.redFront) {
//                startX = -0.944;
//                startY = 0.66;
//                startHeading = 1.578;
//            }
//        }

        while (odo.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY && !Thread.currentThread().isInterrupted()) {
            odo.update();
            telemetry.addData("status", odo.getDeviceStatus());
            telemetry.addData("status2",Thread.currentThread().isInterrupted() );
            telemetry.update();
        }

        odo.setPosition(new Pose2D(DistanceUnit.CM, -0.944 / MODEL_POS_SCALE, 0.66 / MODEL_POS_SCALE, AngleUnit.RADIANS, 1.578));
        currentStage = autoStage.firstShootDrive;
        odo.update();
        try {
            // Load AI model
            model = new BODYONNX(hardwareMap.appContext.getAssets());
            telemetry.addData("Status", "Model loaded successfully");
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
        stateMachine();

        odo.update();

        Pose2D currentPose = odo.getPosition();
        robotX = (currentPose.getX(DistanceUnit.CM) * MODEL_POS_SCALE);
        robotY = (currentPose.getY(DistanceUnit.CM) * MODEL_POS_SCALE);
//        robotX = (Math.abs(currentPose.getX(DistanceUnit.CM) * MODEL_POS_SCALE) + 1) / 2;
//        robotY = (currentPose.getY(DistanceUnit.CM) * MODEL_POS_SCALE) + 1 / 2;
        currentHeading = (float) currentPose.getHeading(AngleUnit.DEGREES);

        // Build observations
        float[] observations = buildObservations();

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

// Divide each component by the denominator.
// If the total is <= 1.0, nothing changes.
// If the total is > 1.0, they are all scaled down proportionally.
        forward /= denominator;
        strafe /= denominator;
        rotation /= denominator;

        drive.setPower( forward * AutoConstants.driveForwardMult, strafe * AutoConstants.driveStrafeMult, rotation * AutoConstants.driveRotationMult);
        spindex.update();
        spindex.shootConsecutive(color);

        autoAim();
        updateColor();
        if(intake.getPower() != 0){
            intakeCheck();
        }
        if(intake.paused) intake.pause(Constants.intakeReverseTime, intakePauseTimer, Intake.intakePower);

        turret.loop();

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        //double heading = currentPose.getHeading(AngleUnit.RADIANS);

        // Convert meters to inches for FTC Dashboard (uses official field frame in inches)
        double robotXInches = (robotX ) * AutoConstants.telemetryScale;
        double robotYInches = (robotY ) * AutoConstants.telemetryScale;
        double targetXInches = targetX * AutoConstants.telemetryScale;
        double targetYInches = targetY * AutoConstants.telemetryScale;

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
        telemetry.addData("Pinpoint Frequency", odo.getFrequency()); //prints/gets the current refresh rate of the Pinpoint
        double newTime = getRuntime();
        double loopTime = newTime-oldTime;
        double frequency = 1/loopTime;
        oldTime = newTime;
        telemetry.addData("REV Hub Frequency: ", frequency); //prints the control system refresh rate
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
        telemetry.addData("Current Auto", AutoConstants.currentAuto);
        telemetry.addData("DTT", DTT);

        telemetry.update();
    }
    public void updateColor(){
        if(!spindex.withinTarget()){
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

        if (!spindex.withinTarget()){
            return;
        }


        if ((detectedColor == "GREEN" || detectedColor == "PURPLE") && spindex.withinTarget()){
            if(autoSortTimerStarted && autoSortTimer.milliseconds() >= Constants.autoSortDelayMs){ // Timer expired: we should sort or spin spindex
                if(purCount + grnCount <= 2){
                    sorted = false;
                    intake.paused = true;
                    intakePauseTimer.reset();
                    autoSortTimer.reset();
                    autoSortTimerStarted = false;
                    intakeCount++;
                    spindex.spinTurns(1);
                    currentPosition = (currentPosition + 1) % 3;
                }
                else{
                    fortelemetry = currentPosition;
                    int turns = autoSort.sortNShoot(currentLayout, detectedMotif                                                                , currentPosition);
                    spindex.spinTurns(turns);
                    telemetry.addData("auto sort", autoSort.sortNShoot(currentLayout, detectedMotif, currentPosition));
                    currentPosition = (currentPosition + turns) % 3;
                    fortelemetry2 = currentPosition;
                    sorted = true;
                    intake.togglePower(Intake.intakePower);
                }
            } else if(!autoSortTimerStarted){ // We should start the timer
                autoSortTimerStarted = true;
                autoSortTimer.reset();
            }
        }
    }

    public void autoAim() {
        LLResult result = turretSpin.limelight.getLatestResult();

        if (result == null || !result.isValid()){
            turretSpin.spinRightCR((float) (lastAutoAimPower * Constants.autoAimLoseMultiplier));
            lastAutoAimPower *= Constants.autoAimLoseDecayMultiplier;
            return;
        }

        if (result.getFiducialResults().get(0).getFiducialId() != Util.getTargetId()){
            if(result.getFiducialResults().get(0).getFiducialId() == 21){
                detectedMotif = "GPP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 22){
                detectedMotif = "PGP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 23){
                detectedMotif = "PPG";
            }
            turretSpin.spinRightCR((float) (lastAutoAimPower * Constants.autoAimLoseMultiplier));
            lastAutoAimPower *= Constants.autoAimLoseDecayMultiplier;
            return;
        }

        double tx = result.getTx();
        double error = tx; //u want tx=0

        double deadband = 1.0; //degrees
        if (Math.abs(error) < deadband) {
            turretSpin.spinRightCR(0);
            return;
        }

        double derivative = error - turretSpin.lastError;
        turretSpin.lastError = error;
        float power = (float) (Constants.limelightKP * error + Constants.limelightKD * derivative);

        turretSpin.lastError = error;

        power = (float) Range.clip(power, -0.75, 0.75);
        lastAutoAimPower = power;
        turretSpin.spinRightCR(power);

        distToAprilTag = result.getBotposeAvgDist();

        hood.setPosition(TurretRegression.getHoodPosition(scaled));
        telemetry.addData("target hood pos", TurretRegression.getHoodPosition(scaled));
        turret.setTargetRPM(TurretRegression.getTurretRPM(scaled));
        telemetry.addData("target turret rpm", TurretRegression.getTurretRPM(scaled));
    }

}
