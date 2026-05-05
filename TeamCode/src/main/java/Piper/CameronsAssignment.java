package Piper;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

public class CameronsAssignment {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    float LeftStick_x;
    float LeftStick_y;
    float RightStick_x;

//I know that technically this doesn't work but I think the logic is there I just don't know how to import stuff.

    public void init(){
        DcMotor FL = hardwareMap.get(class DcMotor);
        DcMotor FR = hardwareMap.get(class DcMotor);
        DcMotor BL = hardwareMap.get(class DcMotor);
        DcMotor BR = hardwareMap.get(class DcMotor);

        float LeftStick_x = Gamepad.get(float left_stick_x);
        float LeftStick_y = Gamepad.get(float left_stick_y);
        float RightStick_x = Gamepad.get(float right_stick_x);

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
