package org.firstinspires.ftc.teamcode.emmy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class Emmydrive extends OpMode {
    DcMotor leftfront;
    DcMotor rightfront;
    DcMotor leftback;
    DcMotor rightback;

    @Override
    public void init() {
        leftfront = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfront = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        leftback = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightback = hardwareMap.get(DcMotor.class, "rightbackdrive");

    }

    void SetDrivePower (){
        float y = gamepad1.left_stick_y;
        float x = gamepad1.left_stick_x;
        float z = gamepad1.right_stick_x;
        float LEFTFRONTpower = (y+x+z);
        float LEFTBACKpower = (y-x+z);
        float RIGHTFRONTpower = (y-x-z);
        float RIGHTBACKpower = (y+x-z);

        leftfront.setPower(LEFTFRONTpower);
        rightfront.setPower(RIGHTFRONTpower);
        leftback.setPower(LEFTBACKpower);
        rightback.setPower(RIGHTBACKpower);
    }

    @Override
    public void loop() {
        SetDrivePower();
    }

    //great work this should work
    //good e
}