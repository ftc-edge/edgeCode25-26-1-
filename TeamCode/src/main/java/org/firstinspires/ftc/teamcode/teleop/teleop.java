package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.automation.TurretAutoAim;
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

    NormalizedColorSensor color;
    NormalizedRGBA colors;
    Intake intake;
    Drive drive;
    Spindex spindex;
    Turret turret;

    TurretSpin turretSpin;
    Hood hood;
    float turretPower;

    Gamepad prevGamepad1 = new Gamepad();
    Gamepad prevGamepad2 = new Gamepad();

    int powerLevel = 4;

    int[] currentLayout = new int[]{0, 0, 0};

    boolean processingBall = false;
    float intakeCount;

    spindexAutoSort.targetMotif target;

    spindexAutoSort autoSort;

    TurretAutoAim aim;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        spindex = new Spindex(hardwareMap);
        hood = new Hood(hardwareMap);
        turretSpin = new TurretSpin(hardwareMap);
        drive = new Drive(hardwareMap);
        autoSort = new spindexAutoSort(hardwareMap);

        color = hardwareMap.get(RevColorSensorV3.class, "color");
        //color.enableLed(true);

        target = spindexAutoSort.targetMotif.GPP;
    }

    @Override
    public void loop() {
        colors = color.getNormalizedColors();
        //color.getNormalizedColors();

        // Switch Power Levels
        handlePowerLevel();

        autoAim();

        // Intake
        if(gamepad1.cross && !prevGamepad1.cross){
            intake.togglePower(1);

        }

        if(intake.getPower() == 1){
            intakeCheck(JavaUtil.colorToHue(colors.toColor()));
            if(intakeCount == 3){
                autoSort.sortNShoot(currentLayout, target);
            }
        }

        //turretSpin.spinRightCR((gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretSpinSpeed);

        turret.setPower(gamepad1.right_trigger);
        if(gamepad1.square){
            hood.setPosition(hood.getPosition() + 0.1f);
        }
        if(gamepad1.circle){
            hood.setPosition(hood.getPosition() - 0.1f);
        }

        // Spindex
        if (gamepad1.dpad_right && !prevGamepad1.dpad_right) {
            spindex.spinUp();
        }
        if (gamepad1.dpad_left && !prevGamepad1.dpad_left) {
            spindex.spinTurns(1);
        }

        if (gamepad1.dpad_up) {
            spindex.stop();
        }

        // Telemetry
        telemetry.addData("Spindex position", spindex.getCurrentPosition());
        telemetry.addData("Spindex Target", spindex.getTargetPosition());
        telemetry.addData("Hood Position", hood.getPosition());
        telemetry.addData("Power Level", powerLevel);
        telemetry.addData("Turret Power", turretPower);
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Processing Ball:", processingBall);
        telemetry.update();

        // Drive
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        Drive.setPower(forward, strafe, pivot);

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
    }

    public void intakeCheck(float hue) {
        if (hue > 100) {
            if (hue < 200 && !processingBall) {
                processingBall = true;
                spindex.spinTurns(1);
                currentLayout[0] = 1;
                currentLayout = new int[]{currentLayout[2], currentLayout[0], currentLayout[1]};
                intakeCount++;
            }else if (hue > 200 && !processingBall) {
                spindex.spinTurns(1);
                currentLayout[0] = -1;
                currentLayout = new int[]{currentLayout[2], currentLayout[0], currentLayout[1]};
                intakeCount++;
            }
        }else{
            processingBall = true;
        }
   }

    public void handlePowerLevel(){
        if (gamepad1.right_bumper && !prevGamepad1.right_bumper) {
            powerLevel++;
        }

        if (gamepad1.left_bumper && !prevGamepad1.left_bumper) {
            powerLevel--;
        }

        powerLevel = Math.min(4, Math.max(powerLevel, 1));

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
        if (result == null || !result.isValid()) {
            turretSpin.spinRightCR(0);
            telemetry.addData("null", 0);
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