package org.firstinspires.ftc.teamcode.automation;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A; //limelight sutff

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; //the usual

import com.qualcomm.robotcore.hardware.CRServo; //continuous roation
import com.qualcomm.robotcore.util.Range; //the clip thing


@TeleOp(name = "Turret Auto Aim", group = "TeleOp")
public class TurretAutoAim extends LinearOpMode {
    private CRServo leftServo;
    private CRServo rightServo;
    private Limelight3A limelight;

    private double lastError = 0;
    private double prevErrorSign = 0;

    @Override
    public void runOpMode() {

        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("u can start now");
        telemetry.update();

        waitForStart();
// GOAL: rotate until tx becomes 0, read limelight

        while (opModeIsActive()) {
            if (gamepad1.left_bumper) {
                autoAim();
            } else {
                leftServo.setPower(0);
                rightServo.setPower(0);
            }
        }
    }

    private void autoAim() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            leftServo.setPower(0);
            rightServo.setPower(0);
            return;
        }
        double tx = result.getTx();
        // if tx is positive (target to the right) the turret needs to rotate right to center therefore -tx
// most motors rotate + pwr = turn right. so we shouldnt need to make it negative??
//using if no target then stop

        //computing how much error:
        double error = -tx; //u want tx=0

//i feel like we should have a deadzone bc its continuous
        //bang on exactly right - cameron
        double deadband = 1.0; //degrees
        if (Math.abs(error) < deadband) {
            leftServo.setPower(0);
            rightServo.setPower(0);
            return;
        }

//controller
// maybe autoaim work
// turretPower = kP*error
// the more the turret is "off target" (big error) more rotate
// as we get close (error small),
// turretPower naturally becomes very small so the motor slows down and settles.
//kD is essentially a dampener

        double derivative = error - lastError;
        lastError = error;
        double kP = 0.02;      // start here tune up or down prob
        double kD = 0.01;
        double power = kP * error + kD * derivative;

//no overshoot

        prevErrorSign = error;

//range/not all power can be used:
        power = Range.clip(power, -0.35, 0.35);

//apply power mirrored to servos bc they spin in opposite directions
        leftServo.setPower(power);
        rightServo.setPower(power);

//telemetrry
        telemetry.addData("tx", tx);
        telemetry.addData("error", error);
        telemetry.addData("power", power);
        telemetry.update();
    }
}
//sources for tx ty https://www.youtube.com/watch?v=xqxvo64xnf4
//sources for tracking https://www.youtube.com/watch?v=-EfOzB_A00Q&t=304s https://www.youtube.com/watch?v=Ap1lBywv00M
//sources for general apriltag tracking not limelight3A https://www.youtube.com/watch?v=OZt33z-lyYo&t=1s

