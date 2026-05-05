package org.firstinspires.ftc.teamcode.iviven;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class fridaynightbtw extends OpMode {
    private static DcMotor LF;
    private static DcMotor LB;
    private static DcMotor RF;
    private static DcMotor RB;

    public void init() {
        DcMotor LF = hardwareMap.dcMotor.get("LF");
        DcMotor LB = hardwareMap.dcMotor.get("LB");
        DcMotor RF = hardwareMap.dcMotor.get("RF");
        DcMotor RB = hardwareMap.dcMotor.get("RB");

        LB.setDirection(DcMotorSimple.Direction.REVERSE);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    double y = -gamepad1.left_stick_y;
    double x = gamepad1.left_stick_x;
    double hx = -gamepad1.right_stick_x;

    double LFM  = x + y + hx;
    double RFM = x - y - hx;
    double LBM   = x - y + hx;
    double RBM  = x + y - hx;



    @Override
    public void loop() {
        /*
    double LFM  = x + y + hx;
    double RFM = x - y - hx;
    double LBM   = x - y + hx;
    double RBM  = x + y - hx;
         */
        LF.setPower(LFM);
        RF.setPower(RFM);
        LB.setPower(LBM);
        RB.setPower(RBM);
    }


}


//g


