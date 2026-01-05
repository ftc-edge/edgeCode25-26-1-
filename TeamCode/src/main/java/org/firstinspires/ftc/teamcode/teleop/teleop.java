package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


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


@TeleOp
public class teleop extends OpMode{

    RevColorSensorV3 color;
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

    boolean processingBall;
    float intakeCount;

    spindexAutoSort.targetMotif target;

    spindexAutoSort autoSort;

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
        color.enableLed(true);

        target = spindexAutoSort.targetMotif.GPP;
    }

    @Override
    public void loop() {
        color.getNormalizedColors();

        // Switch Power Levels
        handlePowerLevel();

        // Intake
        if(gamepad1.cross && !prevGamepad1.cross){
            intake.togglePower(1);
            intakeCheck();
            if(intakeCount == 3){
                autoSort.sortNShoot(currentLayout, target);
            }
        }

        // Turret
        if (gamepad1.a && !prevGamepad1.a) {
            turret.togglePower(turretPower);
        }

        turretSpin.spinRightCR((gamepad1.right_trigger - gamepad1.left_trigger) * Constants.turretSpinSpeed);


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
        telemetry.update();

        // Drive
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        Drive.setPower(forward, strafe, pivot);

        prevGamepad1.copy(gamepad1);
        prevGamepad2.copy(gamepad2);
    }

    public void intakeCheck(){
        if(color.green() > color.blue() && color.green() > 100 && !processingBall){
            processingBall = true;
            spindex.spinTurns(1);
            currentLayout[0] = 1;
            currentLayout = new int[]{currentLayout[2], currentLayout[0], currentLayout[1]};
            intakeCount++;
        }
        else if(color.blue() > color.green()  && color.blue() > 150 && !processingBall){
            spindex.spinTurns(1);
            currentLayout[0] = -1;
            currentLayout = new int[]{currentLayout[2], currentLayout[0], currentLayout[1]};
            intakeCount++;
        }else{
            processingBall = false;
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
                hood.setPosition(Constants.HOOD1);
                turretPower = Constants.TURRET1;
                break;
            case 2:
                telemetry.addData("power", "2");
                gamepad1.setLedColor(235, 225, 70, Gamepad.LED_DURATION_CONTINUOUS);
                hood.setPosition(Constants.HOOD2);
                turretPower = Constants.TURRET2;
                break;
            case 3:
                telemetry.addData("power", "3");
                gamepad1.setLedColor(235, 164, 70, Gamepad.LED_DURATION_CONTINUOUS);
                hood.setPosition(Constants.HOOD3);
                turretPower = Constants.TURRET3;
                break;
            case 4:
                gamepad1.setLedColor(235, 70, 70, Gamepad.LED_DURATION_CONTINUOUS);
                hood.setPosition(Constants.HOOD4);
                turretPower = Constants.TURRET4;
                break;
            default:
                telemetry.addData("Error", "Invalid Power Level");
        }
    }


}