package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp()
public class motorTest extends OpMode {

    private DcMotor motor1;
    private DcMotor motor2;

    private float motorOutput;

    @Override
    public void init() {
        motor1 = hardwareMap.get(DcMotor.class, "motor1");
        motor2 = hardwareMap.get(DcMotor.class, "motor2");
    }

    @Override
    public void loop() {
        motor1.setPower(motorOutput);
        motor2.setPower(-motorOutput);
        motorOutput = gamepad1.left_stick_y;
    }
}
