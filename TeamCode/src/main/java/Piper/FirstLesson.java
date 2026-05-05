package Piper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
public class FirstLesson extends OpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    public void init(){
        DcMotor FL = hardwareMap.get(class DcMotor);
        DcMotor FR = hardwareMap.get(class DcMotor);
        DcMotor BL = hardwareMap.get(class DcMotor);
        DcMotor BR = hardwareMap.get(class DcMotor);
        float speed = 0;
    }

    public void DriveSpeed(float speed){
        FL.setPower(speed);
        FR.setPower(speed);
        BL.setPower(speed);
        BR.setPower(speed);
    }

    public void loop(){
        DriveSpeed(1);
    }
}
