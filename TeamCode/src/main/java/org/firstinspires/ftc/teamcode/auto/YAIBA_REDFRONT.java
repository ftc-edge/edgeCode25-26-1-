package org.firstinspires.ftc.teamcode.auto;

import static java.lang.Thread.sleep;

import android.content.res.AssetManager;

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
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.yaiba.YAIBAONNX;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.util.ElapsedTime;

import ai.onnxruntime.OrtException;

import org.firstinspires.ftc.teamcode.components.AutoRedConstants;

// TODO: intake further
@TeleOp
public class YAIBA_REDFRONT extends OpMode {

    private YAIBAONNX model;

    private GoBildaPinpointDriver odo;

    private double MODEL_POS_SCALE = AutoRedConstants.MODEL_POS_SCALE;
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

    public ElapsedTime jamTimer = new ElapsedTime();

    boolean prevShootingBoolean = false;

    boolean hasPushed = false;

    boolean motifCheck = false;

    boolean adjusted = false;

    public float shootSpeed = AutoRedConstants.shootSpeed;

    public boolean wasAtTarget;

    public ElapsedTime reverseTimer = new ElapsedTime();

    boolean reset = false;
    boolean started = false;
    public ElapsedTime globalTimer = new ElapsedTime();

    // ── Full-tray reverse ────────────────────────────────────────────────────
    // Triggered once the 3rd ball is confirmed seated in intakeCheck().
    // Phase 1: wait FULL_TRAY_SETTLE_DELAY_MS so the ball finishes rolling in.
    // Phase 2: run intake in reverse at FULL_TRAY_REVERSE_POWER for
    //          FULL_TRAY_REVERSE_DURATION_MS to eject any trailing 4th ball.
    // Tune all three values via FTC Dashboard.

    /** Milliseconds to wait after the 3rd ball is confirmed before reversing. */
    public static double FULL_TRAY_SETTLE_DELAY_MS    = 350;

    /** Milliseconds to run the reverse after the settle delay. */
    public static double FULL_TRAY_REVERSE_DURATION_MS = 200;
    public static double AFTER_REVERSE_POWER = 0.5;

    /** Intake power for the gentle reverse. Negative = eject direction. */
    public static double FULL_TRAY_REVERSE_POWER      = -0.35;

    private final ElapsedTime fullTrayReverseTimer = new ElapsedTime();
    private boolean           fullTrayReversePending = false;
    private boolean           fullTrayReverseRunning = false;

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
                targetX = AutoRedConstants.shootX;
                targetY = AutoRedConstants.shootY;
                targetAngle = 1.578f;
                AutoRedConstants.driveForwardMult = 1;
                AutoRedConstants.driveStrafeMult = -1;
                buildObservations();
                if(DTT < 0.05){
                    startShoot = true;
                    scaled = AutoRedConstants.shootScaled1;
                    currentStage = autoStage.firstShot;
                    timer.reset(); // ADDED: start delay timer when entering shoot stage
                }
                break;
            case firstShot:
                intake.setPower(0);
                if(timer.milliseconds() > AutoRedConstants.beforeShootDelayMS) {
                    if (startShoot && turret.atTarget()) {
                        pid.setTargetStep(-4);
                        currentPosition = (currentPosition + 2) % 3; // -4 steps ≡ +2 mod 3
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
                reset = false;
                intake.setPower(1);
                if(timer.milliseconds() > AutoRedConstants.beforeShootDelayMS) {
                    if (!adjusted) {
                        adjusted = pid.shootConsecutiveAdjust();
                    } else if (startShoot && turret.atTarget() && pid.isAtTarget()) {
                        pid.startShootConsecutive();
                        startShoot = false;
                    } else if (pid.isAtTarget() && !startShoot && !pid.shooting) {
                        intakeCheckEnabled = true;
                        pid.shot = 0;
                        sorted = false;
                        shootCnt++;
                        if (shootCnt == 2) currentStage = autoStage.gatePushSetup;
                        if (shootCnt == 3) currentStage = autoStage.thirdPickupSetup;
                        if (shootCnt == 4) currentStage = autoStage.finish;
                        timer.reset();
                    }
                }
                break;

            case firstPickupSetup:
                targetX = AutoRedConstants.intake1PrepX;
                targetY = AutoRedConstants.intakePrepY;
                targetAngle = 1.578f;
                aim.setTargetToMotif();
                intake.setPower(1);           // start intake while driving to position
                intakeCheckEnabled = true;
                autoSortTimerStarted = false; // clear stale timer state from previous cycle
                sorted = false;
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.firstPickup;
                }
                break;

            case firstPickup:
                intake.setPower(1);
                targetX = AutoRedConstants.intake1PrepX;
                targetY = AutoRedConstants.intakeY;
                AutoRedConstants.driveForwardMult = 0.5f;
                AutoRedConstants.driveStrafeMult = -0.5f;
                buildObservations();
                intakeCheckEnabled = true;
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoRedConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                        //intake.togglePower(intake.intakePower);
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    currentStage = autoStage.shootDrive;
                }
                break;

            case shootDrive:
//                intake.setPower(0);
                targetX = AutoRedConstants.shootX;
                targetY = AutoRedConstants.shootY;
                targetAngle = 1.578f;
                AutoRedConstants.driveForwardMult = 1f;
                AutoRedConstants.driveStrafeMult = -1f;
                aim.setTargetToGoal();
                buildObservations();
                if(!reset){
                    reverseTimer.reset();
                    adjusted = false;
                    pid.shot = 0;
                    // Cancel any lingering fullTrayReverse state so it doesn't fight
                    // the in-transit reverse or the shooter sequence.
                    fullTrayReversePending = false;
                    fullTrayReverseRunning = false;
                    reset = true;
                }else if(reverseTimer.seconds() > AutoRedConstants.reverseTime) intake.setPower((float) AutoRedConstants.reversePower);
                if (!adjusted && pid.isAtTarget()) {
                    adjusted = pid.shootConsecutiveAdjust();
                }
                if(DTT < 0.05){
                    startShoot = true;
                    scaled = AutoRedConstants.shootScaled2;
                    currentStage = autoStage.shoot;
                    timer.reset();
                }
                break;

            case gatePushSetup:
                targetX = AutoRedConstants.gatePushPrepX;
                targetY = AutoRedConstants.gatePushPrepY;
                //targetAngle = -2.0944f;
                buildObservations();
                if(DTT< 0.1){
                    currentStage = autoStage.gatePush;
                }
                break;

            case gatePush:
                targetX = AutoRedConstants.gatePushPrepX;
                targetY = AutoRedConstants.gatePushY;
                AutoRedConstants.driveForwardMult = 0.75f;
                AutoRedConstants.driveStrafeMult = -0.75f;
                buildObservations();
                intake.setPower(1);
                if(DTT < 0.05){
                    if(!hasPushed){
                        hasPushed = true;
                        gateTimer.reset();
                    }else if(gateTimer.seconds() < AutoRedConstants.pushTime){
                        currentStage = autoStage.gatePickup;
                    }
                }
                break;
            case gatePickup:
                intake.setPower(1);
                targetX = AutoRedConstants.gatePickupX;
                targetY = AutoRedConstants.gatePickupY;
                AutoRedConstants.driveForwardMult = 0.75f;
                AutoRedConstants.driveStrafeMult = -0.75f;
                targetAngle = (float) AutoRedConstants.gateHeading;
                buildObservations();
                if(DTT < 0.05){
                    //intake.setPower(1);
                    if(!startedTimer){
                        gateTimer.reset();
                        startedTimer = true;
                    }else if(gateTimer.seconds() > AutoRedConstants.gateTime){
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    currentStage = autoStage.shootDrive;
                }
                break;
            case secondPickupSetup:
                targetX = AutoRedConstants.intake2PrepX;
                targetY = AutoRedConstants.intakePrepY;
                AutoRedConstants.driveForwardMult = 1f;
                AutoRedConstants.driveStrafeMult = -1f;
                intake.setPower(1);           // start intake while driving to position
                intakeCheckEnabled = true;
                autoSortTimerStarted = false;
                sorted = false;
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.secondPickup;
                }
                break;

            case secondPickup:
                targetX = AutoRedConstants.intake2PrepX;
                targetY = AutoRedConstants.intakeY;
                AutoRedConstants.driveForwardMult = 0.5f;
                AutoRedConstants.driveStrafeMult = -0.5f;
                intake.setPower(1);
                intakeCheckEnabled = true;
                buildObservations();
                //intake issue
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoRedConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    currentStage = autoStage.shootDrive;
                }
                break;

            case thirdPickupSetup:
                targetX = AutoRedConstants.intake3PrepX;
                targetY = AutoRedConstants.intakePrepY;
                AutoRedConstants.driveForwardMult = 1f;
                AutoRedConstants.driveStrafeMult = -1f;
                intake.setPower(1);           // start intake while driving to position
                intakeCheckEnabled = true;
                autoSortTimerStarted = false;
                sorted = false;
                buildObservations();
                if(DTT < 0.05){
                    currentStage = autoStage.thirdPickup;
                }
                break;

            case thirdPickup:
                targetX = AutoRedConstants.intake3PrepX;
                targetY = AutoRedConstants.intake3y;
                AutoRedConstants.driveForwardMult = 0.5f;
                AutoRedConstants.driveStrafeMult = -0.5f;
                buildObservations();
                intake.setPower(1);
                intakeCheckEnabled = true;
                if(DTT < 0.05){
                    if(!arrived){
                        intakeTimer.reset();
                        arrived = true;
                    }else if(intakeTimer.seconds() > AutoRedConstants.intakeTime){
                        currentStage = autoStage.shootDrive;
                    }
                }
                if(Math.abs(currentLayout[0]) == 1 && Math.abs(currentLayout[1]) == 1 && Math.abs(currentLayout[2]) == 1){
                    currentStage = autoStage.shootDrive;
                }
                break;

            case finish:
                aim.setTargetToInitial();
                targetX = -0.8f;
                targetY = 0.3f;
                targetAngle = -1.578f;
                break;
        }
        //}
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive = new Drive(hardwareMap);
        color = new Color(hardwareMap);
        intake = new Intake(hardwareMap);

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        autoSort = new SpindexAutoSort(hardwareMap, telemetry);

        pid = new SpindexPID(hardwareMap);

        aim = new TurretAutoAimODO(hardwareMap, -0.944, -0.66, "auto");

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

        odo.setPosition(new Pose2D(DistanceUnit.CM, -0.944 / MODEL_POS_SCALE, 0.66 / MODEL_POS_SCALE, AngleUnit.RADIANS, 1.578f));
        currentStage = autoStage.firstShootDrive;
        try {
            // Load AI model
            model = new YAIBAONNX(hardwareMap.appContext.getAssets());
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

        /* Logic for terminating after 30 seconds */
        if(!started){
            started = true;
            globalTimer.reset();
        }
        if(started && globalTimer.seconds() > 26){
            currentStage = autoStage.finish;
        }

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

        drive.setPower(forward * AutoRedConstants.driveForwardMult * AutoRedConstants.driveMult, strafe * AutoRedConstants.driveStrafeMult * AutoRedConstants.driveMult, rotation * AutoRedConstants.driveRotationMult * AutoRedConstants.driveMult);
        pid.update();

        aim.runToAim(telemetry);

        pid.shootConsecutive(color);

        //motifCheck();


        updateColor();
        if(intake.getPower() != 0 && intakeCheckEnabled && !sorted && !fullTrayReversePending && !fullTrayReverseRunning){
            intakeCheck();
        }
        if(intake.paused) intake.pause(Constants.intakeReverseTime, intakePauseTimer, Intake.intakePower);

        jamCorrection();
        fullTrayReverse();


        wasAtTarget = pid.isAtTarget();

        hood.setPosition(AutoRedConstants.hoodPos);
        turret.setTargetRPM(shootSpeed);
        telemetry.addData("target hood pos", hood.getPosition());
        telemetry.addData("target turret rpm", 140);

        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Convert meters to inches for FTC Dashboard (uses official field frame in inches)
        double robotXInches = (robotX ) * AutoRedConstants.telemetryScale;
        double robotYInches = (robotY ) * AutoRedConstants.telemetryScale;
        double targetXInches = targetX * AutoRedConstants.telemetryScale;
        double targetYInches = targetY * AutoRedConstants.telemetryScale;

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
        //telemetry.addData("Current Auto", AutoRedConstants.currentAuto);
        telemetry.addData("DTT", DTT);
        telemetry.addData("Shooting", pid.shooting);
        telemetry.addData("Sorted", sorted);
        telemetry.addData("IntakeCheckEnabled", intakeCheckEnabled);
        telemetry.addData("CurrentPosition", currentPosition);
        telemetry.addData("Layout", currentLayout[0] + " " + currentLayout[1] + " " + currentLayout[2]);

        telemetry.addData("=== COLOR ===", "");
        telemetry.addData("Color", color.getColor());
        telemetry.addData("Full Tray Settle", fullTrayReversePending);
        telemetry.addData("Full Tray Reversing", fullTrayReverseRunning);


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
        else if("GREEN".equals(color.getColor())) {
            currentLayout[currentPosition] = 1;
        }
        else if("PURPLE".equals(color.getColor())) {
            currentLayout[currentPosition] = -1;
        }
        else if("NONE".equals(color.getColor())){
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

        if (("GREEN".equals(detectedColor) || "PURPLE".equals(detectedColor)) && pid.isAtTarget()){
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
                    pid.shot = 0;
                    autoSortTimerStarted = false;
                    autoSortTimer.reset();
                    intake.togglePower(Intake.intakePower);
                    // Start the settle-then-reverse sequence now that the 3rd ball is confirmed.
                    fullTrayReversePending = true;
                    fullTrayReverseRunning = false;
                    fullTrayReverseTimer.reset();
                }
            } else if(!autoSortTimerStarted){
                autoSortTimerStarted = true;
                autoSortTimer.reset();
            }
        }
    }

    /**
     * Two-phase gentle reverse after the 3rd ball is confirmed seated.
     *
     * Phase 1 — Settle: waits FULL_TRAY_SETTLE_DELAY_MS after the trigger so
     *            the ball that just entered has time to roll fully into the chamber
     *            before anything happens.
     * Phase 2 — Eject:  runs the intake at FULL_TRAY_REVERSE_POWER for
     *            FULL_TRAY_REVERSE_DURATION_MS to push any trailing 4th ball back
     *            out without disturbing the 3 already seated.
     *
     * Call once per loop(). Triggered by setting fullTrayReversePending = true.
     * Cancels automatically if the robot transitions to shoot (pid.shooting).
     */
    public void fullTrayReverse() {
        // Cancel cleanly if shooting has started — don't fight the shooter sequence.
        if (pid.shooting) {
            fullTrayReversePending = false;
            fullTrayReverseRunning = false;
            return;
        }

        if (fullTrayReversePending) {
            // Phase 1: waiting for the ball to fully seat before we touch intake.
            if (fullTrayReverseTimer.milliseconds() >= FULL_TRAY_SETTLE_DELAY_MS) {
                intake.setPower((float) FULL_TRAY_REVERSE_POWER);
                fullTrayReversePending = false;
                fullTrayReverseRunning = true;
                fullTrayReverseTimer.reset();
            }
        } else if (fullTrayReverseRunning) {
            // Phase 2: running the reverse — stop once the duration expires.
            if (fullTrayReverseTimer.milliseconds() >= FULL_TRAY_REVERSE_DURATION_MS) {
                intake.setPower((float) AFTER_REVERSE_POWER);
                fullTrayReverseRunning = false;
            }
        }
    }

    public void jamCorrection(){
//        if(!pid.isAtTarget() && wasAtTarget){
//            jamTimer.reset();
//        }else if(!pid.isAtTarget() && jamTimer.seconds() > 1.5){
//            pid.stop();
//            pid.spin(pid.getCurrentPosition() - pid.getTargetPosition());
//        }
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