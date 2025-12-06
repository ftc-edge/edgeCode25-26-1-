package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;


@Config
public class Hood {

    Servo hoodServo;

    public Hood(HardwareMap hardwareMap){
        hoodServo = hardwareMap.get(Servo.class, "hood");
    }

    public void setPosition(double position){
        hoodServo.setPosition(position);
    }

    public void incrementPosition(double increment){
        hoodServo.setPosition(hoodServo.getPosition() + increment);
    }

    public double getPosition(){
        return hoodServo.getPosition();
    }
}
