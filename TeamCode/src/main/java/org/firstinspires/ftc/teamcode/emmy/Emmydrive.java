package org.firstinspires.ftc.teamcode.emmy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Emmydrive extends OpMode {
    DcMotor leftfront;
    DcMotor rightfront;
    DcMotor leftback;
    DcMotor rightback;

    float speed = 0;

    @Override
    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfront = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        leftback = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightback = hardwareMap.get(DcMotor.class, "rightbackdrive");

    }

    void SetDrivePower (float speed){
        leftfront.setPower(speed);
        rightfront.setPower(speed);
        leftback.setPower(speed);
        rightback.setPower(speed);
    }

    @Override
    public void loop() {
        SetDrivePower(1);
    }
}