package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Spin {
    private CRServo TurretX1;
    private CRServo TurretX2;

    public Spin(HardwareMap hardwareMap){
        TurretX1 = hardwareMap.get(CRServo.class, "TurretX1");
        TurretX2 = hardwareMap.get(CRServo.class, "TurretX2");
    }

    public void setPower(float power){
        TurretX1.setPower(power);
        TurretX2.setPower(-power);
    }
}

//How this is supposed the work:
//Step 1: See if button is being pressed
//Step 2: If button is being pressed, move turret in corresponding direction
//Step 3: Turret Continues moving while button pressed
//Step 4: When button stops being pressed, stop moving
//Step 5: Repeat for both buttons/directions
