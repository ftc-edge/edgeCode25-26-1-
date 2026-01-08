package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

@Config
public class TurretSpin {
    CRServo leftServo;
    CRServo rightServo;

    public Limelight3A limelight;

    public double lastError;
    public double prevErrorSign;

    double targetPosition;
//    public static float spinPower = 0.2f;

    public TurretSpin(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void spinRightCR(float spinPower){
        leftServo.setPower(spinPower);
        rightServo.setPower(spinPower);
    }

    public void spinLeftCR(float spinPower){
        leftServo.setPower(-spinPower);
        rightServo.setPower(-spinPower);
    }
}
