package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.components.Constants.targetId;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.automation.TurretAutoAim;
import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.ColorSamplerUtil;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Spindex;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretSpin;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.automation.spindexAutoSort;


import com.qualcomm.hardware.rev.RevColorSensorV3;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
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

    int[] currentLayout = new int[]{0, 0, 0};
    int currentPosition = 0;

    boolean processingBall = false;
    float intakeCount;

    String detectedColor;

    spindexAutoSort.targetMotif target;

    spindexAutoSort autoSort;

    String detectedMotif = "None Detected";

    private float shootSpeed = Constants.TURRET1;

    boolean sorted = false;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        drive = new Drive(hardwareMap);
        autoSort = new spindexAutoSort(hardwareMap, telemetry);

        color = new Color(hardwareMap);

//        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");
//        colorSensor.enableLed(true);

        target = spindexAutoSort.targetMotif.GPP;
    }

    @Override
    public void loop() {

//        colors = colorSensor.getNormalizedColors();

        // Switch Power Levels
        //handlePowerLevel();

        turret.setPower(shootSpeed);
        hood.setPosition(hoodPosition);

        autoAim();
        updateColor();
        spindex.updateTimer();

        // Intake
        if(gamepad1.cross && !prevGamepad1.cross){
            intake.togglePower(1);
        }

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

        //turretSpin.spinRightCR((gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretSpinSpeed);

        if(gamepad1.right_bumper){
            shootSpeed += 0.004f;
        }
        if(gamepad1.left_bumper){
            shootSpeed -= 0.004f;
        }
        shootSpeed = max(0, min(1, shootSpeed));
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
        spindex.shootConsecutive();
        if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
            spindex.spinTurns(1);
            currentPosition = (currentPosition + 1 ) % 3;
        }
        if (gamepad1.dpad_up  && !prevGamepad1.dpad_up) {
            spindex.stop();
        }

        // Telemetry
        telemetry.addData("Spindex position", spindex.getCurrentPosition());
        telemetry.addData("Spindex Target", spindex.getTargetPosition());
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Shoot Speed", shootSpeed);
        telemetry.addData("Intake Count", intakeCount);
        telemetry.addData("Turret Power", turretPower);
//        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Processing Ball:", processingBall);
        telemetry.addData("Ball Colors", "%s, %s, %s", numberToColor(currentLayout[0]), numberToColor(currentLayout[1]), numberToColor(currentLayout[2]));
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Spindex is within target", spindex.withinTarget());
        telemetry.addData("Detected Motif", detectedMotif);

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
        if((grnCount + purCount) >= 3){
            return;
        }


        if (!spindex.withinTarget()){
            return;
        }
        if (detectedColor == "NONE") {
            processingBall = false;
        }
        else if ((detectedColor == "GREEN" || detectedColor == "PURPLE") && spindex.withinTarget() && !processingBall) {
            processingBall = true;
            spindex.spinTurns(1);
            currentPosition = (currentPosition + 1 ) % 3;
            intakeCount++;
        }
   }

    public void handlePowerLevel(){
        if (gamepad1.right_bumper && !prevGamepad1.right_bumper) {
            powerLevel++;
        }

        if (gamepad1.left_bumper && !prevGamepad1.left_bumper) {
            powerLevel--;
        }

        powerLevel = min(4, max(powerLevel, 1));

        switch (powerLevel) {
            case 1:
                telemetry.addData("power", "1");
                gamepad1.setLedColor(140, 235, 70, Gamepad.LED_DURATION_CONTINUOUS);
                //hood.setPosition(Constants.HOOD1);
                turretPower = Constants.TURRET1;
                break;
            case 2:
                telemetry.addData("power", "2");
                gamepad1.setLedColor(235, 225, 70, Gamepad.LED_DURATION_CONTINUOUS);
               // hood.setPosition(Constants.HOOD2);
                turretPower = Constants.TURRET2;
                break;
            case 3:
                telemetry.addData("power", "3");
                gamepad1.setLedColor(235, 164, 70, Gamepad.LED_DURATION_CONTINUOUS);
                //hood.setPosition(Constants.HOOD3);
                turretPower = Constants.TURRET3;
                break;
            case 4:
                gamepad1.setLedColor(235, 70, 70, Gamepad.LED_DURATION_CONTINUOUS);
                //hood.setPosition(Constants.HOOD4);
                turretPower = Constants.TURRET4;
                break;
            default:
                telemetry.addData("Error", "Invalid Power Level");
        }
    }

    public void autoAim() {
        LLResult result = turretSpin.limelight.getLatestResult();

        if (result == null || !result.isValid()){
            turretSpin.spinRightCR(0);
            return;
        }

        if (result.getFiducialResults().get(0).getFiducialId() != targetId){
            if(result.getFiducialResults().get(0).getFiducialId() == 21){
                target = spindexAutoSort.targetMotif.GPP;
                detectedMotif = "GPP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 22){
                target = spindexAutoSort.targetMotif.PGP;
                detectedMotif = "PGP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 23){
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

//controller
// maybe autoaim work
// turretPower = kP*error
// the more the turret is "off target" (big error) more rotate
// as we get close (error small),
// turretPower naturally becomes very small so the motor slows down and settles.
//kD is essentially a dampener

        double derivative = error - turretSpin.lastError;
        turretSpin.lastError = error;
        double kP = 0.02;      // start here tune up or down prob
        double kD = 0.01;
        float power = (float) (kP * error + kD * derivative);

//no overshoot

       turretSpin.lastError = error;

//range/not all power can be used:
        power = (float) Range.clip(power, -0.35, 0.35);

//apply power mirrored to servos bc they spin in opposite directions
        turretSpin.spinRightCR(power);

//telemetrry

    }
}