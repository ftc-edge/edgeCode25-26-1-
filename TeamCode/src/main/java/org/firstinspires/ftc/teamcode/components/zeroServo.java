package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class zeroServo extends OpMode {
    Servo servo;

    float servoPosition;
    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
    }

    @Override
    public void loop() {
        servo.setPosition(servoPosition);

        servoPosition += gamepad1.left_trigger * 0.01f;

        servoPosition -= gamepad1.right_trigger * 0.01f;
    }
}
