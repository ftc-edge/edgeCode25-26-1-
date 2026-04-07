package org.firstinspires.ftc.teamcode.ashton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class ashtonclass extends OpMode {

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    @Override
    public void init() {
    }

    @Override
    public void loop() {
        
while(gamepad1.x){
    leftFront.setPower(1);
    rightFront.setPower(1);
    leftBack.setPower(1);
    rightBack.setPower(1);
    }
}
}



