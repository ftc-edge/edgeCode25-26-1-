package org.firstinspires.ftc.teamcode.yizhou;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class intro extends OpMode {
    float p = 0
    DcMotor motor1;
    DcMotor motor2;
    DcMotor motor3;
    DcMotor motor4;
    @Override
    public void init() {
    motor1 = new DcMotor();
    motor2 = new DcMotor();
    motor3 = new DcMotor();
    motor4 = new DcMotor();
    }
    @Override
    public void loop() {
    setDrivePower(p);
    }
    public void setDrivePower (float p) {
        motor1.setPower(p);
        motor2.setPower(p);
        motor3.setPower(p);
        motor4.setPower(p);
}
    }

    
}
