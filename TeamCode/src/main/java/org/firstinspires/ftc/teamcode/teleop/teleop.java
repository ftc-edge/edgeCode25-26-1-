package org.firstinspires.ftc.teamcode.teleop;

import static java.lang.Math.PI;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.components.AutoBlueConstants;
import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.SpindexPID;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretAutoAimODO;
import org.firstinspires.ftc.teamcode.components.TurretRTP;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Util;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.automation.SpindexAutoSort;
import org.firstinspires.ftc.teamcode.components.TurretRegression;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
TODO: Spindex PID
TODO: Limelight Tracking Fix
TODO: Odometry
TODO: Regression
TODO: Ensure autosort works
 */

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

    TurretAutoAimODO aim;

    SpindexPID pid;
    Hood hood;
    float hoodPosition = Constants.HOOD1;
    float turretPower;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    public int[] currentLayout = new int[]{0, 0, 0};
    int currentPosition = 0;

    float intakeCount;

    String detectedColor;

    SpindexAutoSort autoSort;

    private double shootSpeed = Constants.shootSpeed;
    boolean flywheel = true;
    public boolean sorted    = false;
    boolean        adjusting = false;  // true while shootConsecutiveAdjust is running
    int teleopSortStage = 0;

    public ElapsedTime autoSortTimer = new ElapsedTime();
    public ElapsedTime intakePauseTimer = new ElapsedTime();
    boolean autoSortTimerStarted = false;

    int fortelemetry;
    int fortelemetry2;

    boolean prevShootingBoolean = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        drive = new Drive(hardwareMap);
        autoSort = new SpindexAutoSort(hardwareMap, telemetry);
        pid = new SpindexPID(hardwareMap);
        color = new Color(hardwareMap);

        aim = new TurretAutoAimODO(hardwareMap, 0, 0.5, "teleop");

    }

    @Override
    public void loop() {

        shootSpeed += (gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretAdjustSpeed;

        if(gamepad1.dpad_right){
            hoodPosition = Math.min(1, hoodPosition + 0.01f);
        }
        if(gamepad1.dpad_left){
            hoodPosition = Math.max(0, hoodPosition - 0.01f);
        }

        turret.loop();
        pid.update();
        aim.runToAim(telemetry);

        //turretSpin.autoAim();
        updateColor();
        if(intake.getPower() != 0){
            intakeCheck();
        }

        if(intake.paused) intake.pause(Constants.intakeReverseTime, intakePauseTimer, Intake.intakePower);

        //turretSpin.spinRightCR((gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretSpinSpeed);

        // Spindex
        if (gamepad1.right_bumper && !prevGamepad1.right_bumper) {
            teleopSortStage = 1;
            sorted = false;
            pid.shootConsecutiveAdjust();
        }
        if(teleopSortStage == 1){
            boolean finishedAdjusting = pid.shootConsecutiveAdjust();
            if (finishedAdjusting) {
                pid.startShootConsecutive();
                teleopSortStage = 0;
            }
        }

        pid.shootConsecutive(color);

        if (gamepad1.left_bumper && !prevGamepad1.left_bumper) {
            pid.setTargetStep(1);
            currentPosition = (currentPosition + 1 ) % 3;
        }
        if (gamepad1.circle && !prevGamepad1.circle) {
            pid.stop();
        }

        if(gamepad1.dpad_up){
            aim.adjustOffset(Constants.aimAdjust);
        }
        if(gamepad1.dpad_down) {
            aim.adjustOffset(-Constants.aimAdjust);
        }

        // Intake
        if(gamepad1.cross && !prevGamepad1.cross){
            intake.togglePower(Intake.intakePower);
        }
        if(gamepad1.triangle && !prevGamepad1.triangle){
            intake.togglePower(-Intake.intakePower);
        }

        if(gamepad1.square && !prevGamepad1.square){
            flywheel = !flywheel;
        }
        if (flywheel){
            hood.setPosition(hoodPosition);
//            double scaled = AutoBlueConstants.shootScaled3;
//            hood.setPosition(TurretRegression.getHoodPosition(scaled));
            telemetry.addData("current hood pos", hood.getPosition());
            turret.setTargetRPM(shootSpeed);
            telemetry.addData("current turret rpm", turret.getCurrentRPM());
            telemetry.addData("target turret rpm", shootSpeed);
        } else {
            turret.stop();
        }



        // Telemetry
//        telemetry.addData("Spindex position", spindex.getCurrentPosition());
//        telemetry.addData("Spindex Target", spindex.getTargetPosition());
        telemetry.addData("Sorted", sorted);
        telemetry.addData("Adjusting", adjusting);
        telemetry.addData("Alliance Color", AutoBlueConstants.allianceColor);
        telemetry.addData("Distance to April Tag", turretSpin.distToAprilTag);
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Auto Sort Telemetry", "Original Pos " + fortelemetry + " -> " + fortelemetry2);
        telemetry.addData("Turret Current RPM", turret.getCurrentRPM());
        telemetry.addData("Turret Power", turret.getPower());
        telemetry.addData("Turret At Point", turret.getAtPoint());
        telemetry.addData("Shoot Speed", shootSpeed);
        telemetry.addData("Turret Power", turretPower);
//        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Ball Colors", "%s, %s, %s", numberToColor(currentLayout[0]), numberToColor(currentLayout[1]), numberToColor(currentLayout[2]));
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Spindex is within target", pid.isAtTarget());
        telemetry.addData("Detected Motif", turretSpin.detectedMotif);
        telemetry.addData("Linelight Error", turretSpin.lastError);
        //telemetry.addData("purple + green count", )

        telemetry.addData("Mean6",   "(%.1f°, %.1f%%, %.1f%%)", color.getHSL()[0], color.getHSL()[1], color.getHSL()[2]);
        telemetry.addData("Detected Color", detectedColor);
        telemetry.addData("TargetID", Util.getTargetId());

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
            if(autoSortTimerStarted && autoSortTimer.milliseconds() >= Constants.autoSortDelayMs){ // Timer expired: we should sort or spin spindex
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
                    pid.shot = 0;           // reset adjust state machine so it starts from nudge step 0
                    autoSortTimerStarted = false;  // prevent re-triggering if intakeCheck runs again
                    autoSortTimer.reset();
                    intake.togglePower(Intake.intakePower);
                }
            } else if(!autoSortTimerStarted){ // We should start the timer
                autoSortTimerStarted = true;
                autoSortTimer.reset();
            }
        }
    }



}