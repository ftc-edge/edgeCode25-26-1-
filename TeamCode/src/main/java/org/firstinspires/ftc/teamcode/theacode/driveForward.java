package org.firstinspires.ftc.teamcode.theacode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class driveForward extends OpMode {
    private static DcMotor leftFront;
    private static DcMotor leftBack;
    private static DcMotor rightFront;
    private static DcMotor rightBack;
    double speed = 0.0;

    private void setDrivePower(){
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(speed);
        rightBack.setPower(speed);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        setDrivePower();
        speed += .01;
    }
}
