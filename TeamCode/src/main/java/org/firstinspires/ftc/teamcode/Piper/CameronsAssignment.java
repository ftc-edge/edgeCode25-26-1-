package org.firstinspires.ftc.teamcode.Piper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class CameronsAssignment extends OpMode{
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    float LeftStick_x;
    float LeftStick_y;
    float RightStick_x;

//I know that technically this doesn't work but I think the logic is there I just don't know how to import stuff.

    public void init(){
        DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
        DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
        DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
        DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

        float LeftStick_x = gamepad1.left_stick_x;
        float LeftStick_y = gamepad1.left_stick_y;
        float RightStick_x = gamepad1.right_stick_x;

    }

    public void Drive(){
        FL.setPower(LeftStick_x + LeftStick_y + RightStick_x);
        FR.setPower(-LeftStick_x + LeftStick_y - RightStick_x);
        BL.setPower(LeftStick_x + LeftStick_y + RightStick_x);
        BR.setPower(-LeftStick_x + LeftStick_y - RightStick_x);
    }

    public void loop(){
        Drive();
    }
}

// good job piper we will implement the hardware map on tuesday :)
// Goo dE
