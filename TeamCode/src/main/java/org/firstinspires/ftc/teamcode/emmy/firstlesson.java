package org.firstinspires.ftc.teamcode.emmy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


    public class firstlesson extends OpMode {
        DcMotor testMotor;
        DcMotor funMotor;
        DcMotor meanMotor;
        DcMotor niceMotor;


        @Override
        public void init() {
            testMotor = new DcMotor();
            funMotor = new DcMotor();
            meanMotor = new DcMotor();
            niceMotor = new DcMotor();

        }

        @Override
        public void loop() {
            SetDrivePower(1);
        }
        void SetDrivePower (float P){
            testMotor.setPower(P);
            funMotor.setPower(P);
            meanMotor.setPower(P);
            niceMotor.setPower(P);
    }
}


