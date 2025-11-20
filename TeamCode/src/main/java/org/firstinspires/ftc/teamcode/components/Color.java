package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
public class Color{

    private RevColorSensorV3 color;

    public Color(HardwareMap hardwareMap){
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        color.enableLed(true);
    }

    public String getColor(){
        color.getNormalizedColors();
        // Green
        if(color.green() > color.blue() && color.green() > color.red()){
            return "Green";
        }
        if(color.blue() > color.green() && color.red() > color.green()){
            return "Purple";
        }
        return "None";
    }

}
