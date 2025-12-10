package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretSpin {
    CRServo spinMotor1;
    CRServo spinMotor2;

    double targetPosition;
//    public static float spinPower = 0.2f;

    public TurretSpin(HardwareMap hardwareMap) {
        spinMotor1 = hardwareMap.get(CRServo.class, "turretSpin1");
        spinMotor1.setDirection(DcMotorSimple.Direction.FORWARD);

        spinMotor2 = hardwareMap.get(CRServo.class, "turretSpin2");
        spinMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void spinRightCR(float spinPower){
        spinMotor1.setPower(spinPower);
        spinMotor2.setPower(spinPower);
    }

    public void spinLeftCR(float spinPower){
        spinMotor1.setPower(-spinPower);
        spinMotor2.setPower(-spinPower);
    }
}
