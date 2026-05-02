package org.firstinspires.ftc.teamcode.theacode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class theaGamepadAssignment extends OpMode{
    DcMotor leftFront = hardwareMap.dcMotor.get("leftFront");
    DcMotor leftBack = hardwareMap.dcMotor.get("leftBack");
    DcMotor rightFront = hardwareMap.dcMotor.get("rightFront");
    DcMotor rightBack = hardwareMap.dcMotor.get("rightBack");

    public void init(){
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        double LFPower = (y+x+rx);
        double LBPower = (y-x+rx);
        double RFPower = (y-x-rx);
        double RBPower = (y+x-rx);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
        rightFront.setPower(RFPower);
        rightBack.setPower(RBPower);
    }

}
