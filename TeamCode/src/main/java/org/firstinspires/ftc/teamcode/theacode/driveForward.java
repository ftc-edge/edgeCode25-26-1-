package org.firstinspires.ftc.teamcode.theacode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class driveForward extends OpMode {
    private static DcMotor leftFront;
    private static DcMotor leftBack;
    private static DcMotor rightFront;
    private static DcMotor rightBack;

    private void forward(){
        leftFront.setPower(1);
        leftBack.setPower(1);
        rightFront.setPower(1);
        rightBack.setPower(1);
    }

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        forward();
    }
}
