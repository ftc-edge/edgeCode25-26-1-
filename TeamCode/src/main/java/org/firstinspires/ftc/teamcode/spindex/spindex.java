package org.firstinspires.ftc.teamcode.spindex;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class spindex {
    int currentPosition = 1;

    DcMotor spinMotor;
    public spindex(HardwareMap hardwareMap){
        spinMotor = hardwareMap.get(DcMotor.class, "spindex");
        spinMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spinMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void spinRight(){
        // if a pressed
        spinMotor.setTargetPosition(spinMotor.getCurrentPosition() + 60);
        currentPosition = (currentPosition+1) % 3;
    }
}
