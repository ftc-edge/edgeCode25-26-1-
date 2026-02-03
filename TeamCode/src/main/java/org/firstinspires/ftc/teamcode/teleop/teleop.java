package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.components.Constants.targetId;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.automation.SpindexAutoSort;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class teleop extends OpMode{

    //    NormalizedColorSensor colorSensor;
//    NormalizedRGBA colors;
    Intake intake;
    Drive drive;
    Spindex spindex;
    Turret turret;

    Color color;

    TurretSpin turretSpin;
    Hood hood;
    float hoodPosition = Constants.HOOD1;
    float turretPower;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    int powerLevel = 4;

    public int[] currentLayout = new int[]{0, 0, 0};
    int currentPosition = 0;

    float intakeCount;

    String detectedColor;

    SpindexAutoSort.targetMotif target;

    SpindexAutoSort autoSort;

    String detectedMotif = Constants.defaultMotif;

    private float shootSpeed = 0;
        //Turret.targetRPM1;

    public boolean sorted = false;

    public ElapsedTime autoSortTimer = new ElapsedTime();
    public ElapsedTime intakePauseTimer = new ElapsedTime();
    boolean autoSortTimerStarted = false;

    float deltaTime;

    float cameraBuffer = 0;
    int fortelemetry;
    int fortelemetry2;

    double lastAutoAimPower = 0;

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

//        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
//        colorSensor.enableLed(true);

        target = SpindexAutoSort.targetMotif.GPP;
    }

    @Override
    public void loop() {

//        colors = colorSensor.getNormalizedColors();

        // Switch Power Levels
        //handlePowerLevel();

        shootSpeed += (gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretAdjustSpeed;

        turret.loop();
        turret.setTargetRPM(shootSpeed);
        hood.setPosition(hoodPosition);
        spindex.updateTimer();

        autoAim();
        updateColor();

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
        }

        if(intake.paused) intake.pause(Constants.intakeReverseTime, intakePauseTimer, Intake.intakePower);

        //turretSpin.spinRightCR((gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretSpinSpeed);

        if(gamepad1.square){
            hoodPosition += 0.005f;
        }
        if(gamepad1.circle){
            hoodPosition -= 0.005f;
        }

        // Spindex
        if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
            sorted = false;
            spindex.startShootConsecutive();
            currentPosition = (currentPosition + 2 ) % 3;
            currentLayout[currentPosition] = 0;
        }
        spindex.shootConsecutive(color);
        if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
            spindex.spinTurns(1);
            currentPosition = (currentPosition + 1 ) % 3;
        }
        if (gamepad1.dpad_up  && !prevGamepad1.dpad_up) {
            spindex.stop();
        }

        // Intake
        if(gamepad1.cross && !prevGamepad1.cross){
            intake.togglePower(Intake.intakePower);
        }
        if(gamepad1.triangle && !prevGamepad1.triangle){
            intake.togglePower(-Intake.intakePower);
        }


        // Telemetry
//        telemetry.addData("Spindex position", spindex.getCurrentPosition());
//        telemetry.addData("Spindex Target", spindex.getTargetPosition());
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Auto Sort Telemetry", "Original Pos " + fortelemetry + " -> " + fortelemetry2);
        telemetry.addData("Turret Current RPM", turret.getCurrentRPM());
        telemetry.addData("Turret Target RPM", turret.getTargetRPM());
        telemetry.addData("Turret Power", turret.getPower());
        telemetry.addData("Turret At Point", turret.getAtPoint());
        telemetry.addData("Shoot Speed", shootSpeed);
        telemetry.addData("Intake Count", intakeCount);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Auto Sort Timer Started", autoSortTimerStarted);
        telemetry.addData("Auto Sort Timer", autoSortTimer.milliseconds());
//        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Ball Colors", "%s, %s, %s", numberToColor(currentLayout[0]), numberToColor(currentLayout[1]), numberToColor(currentLayout[2]));
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Spindex is within target", spindex.withinTarget());
        telemetry.addData("Detected Motif", target);
        //telemetry.addData("purple + green count", )

        telemetry.addData("Mean6",   "(%.1f°, %.1f%%, %.1f%%)", color.getHSL()[0], color.getHSL()[1], color.getHSL()[2]);
        telemetry.addData("Detected Color", detectedColor);

        telemetry.update();

        // Drive
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        Drive.setPower(forward, strafe, pivot);

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
    }

    public String numberToColor(int x){
        if (x == -1) {
            return "Purple";
        } else if (x == 0) {
            return "None";
        } else if (x == 1) {
            return "Green";
        }
        return "Null";
    }
    public void updateColor(){
        if(spindex.isBusy){
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

        if (result.getFiducialResults().get(0).getFiducialId() != targetId){
            if(result.getFiducialResults().get(0).getFiducialId() == 21){
                target = SpindexAutoSort.targetMotif.GPP;
                detectedMotif = "GPP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 22){
                target = SpindexAutoSort.targetMotif.PGP;
                detectedMotif = "PGP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 23){
                target = SpindexAutoSort.targetMotif.PPG;
                detectedMotif = "PPG";
            }
            return;
        }

        double tx = result.getTx();
        //computing how much error:
        double error = tx; //u want tx=0

//i feel like we should have a deadzone bc its continuous
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

//telemetrry

    }
}