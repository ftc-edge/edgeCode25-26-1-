package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class motorTest extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    private Servo canopyServo;

    private float motorOutput = 0f;
    private float servoPosition = 0f;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
        canopyServo = hardwareMap.get(Servo.class, "canopy");
    }

    @Override
    public void loop() {
        if (gamepad1.x){
            servoPosition += .002f;
        }
        if (gamepad1.y){
            servoPosition -= .002f;
        }
        if (gamepad1.a){
            motorOutput = .6f;
        }
        if (gamepad1.b){
            motorOutput = .63f;
        }
        if (gamepad1.right_bumper){
            motorOutput = .66f;
        }
        if (gamepad1.left_bumper) {
            motorOutput = .7f;
        }

        motorOutput = Math.min(motorOutput, 1f);
        motorOutput = Math.max(motorOutput, 0f);

        servoPosition = Math.min(servoPosition, 1f);
        servoPosition = Math.max(servoPosition, 0f);

        motor1.setPower(motorOutput);
        motor2.setPower(-motorOutput);
        canopyServo.setPosition(servoPosition);

        telemetry.addData("power", motorOutput);
        telemetry.addData("servoPosition", servoPosition);
    }
}

//0.4~0.5 power