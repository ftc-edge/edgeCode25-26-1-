package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;


@Config
public class Turret {
    private DcMotor TurretX1;
    private DcMotor TurretX2;

    public static int disableShoot1 = 0;
    public static int disableShoot2 = 0;

    public Turret(HardwareMap hardwareMap){
        if(disableShoot1 == 0){
            TurretX1 = hardwareMap.get(DcMotor.class, "motor1");
            TurretX1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TurretX1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TurretX1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            TurretX1.setDirection(DcMotor.Direction.FORWARD);
        }

        if(disableShoot2 == 0){
            TurretX2 = hardwareMap.get(DcMotor.class, "motor2");
            TurretX2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            TurretX2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            TurretX2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            TurretX2.setDirection(DcMotor.Direction.FORWARD);
        }

    }

    public void setPower(float power){
        if(disableShoot1 == 0){
            TurretX1.setPower(power);
        }
        if(disableShoot2 == 0){
            TurretX2.setPower(power);
        }
    }

    public void togglePower(float power) {
        if(disableShoot1 == 0){
            if (TurretX2.getPower() == 0) {
                this.setPower(power);
            } else {
                this.setPower(0);
            }
        } else if(disableShoot2 == 0){
            if (TurretX1.getPower() == 0) {
                this.setPower(power);
            } else {
                this.setPower(0);
            }
        } else {
            this.setPower(0);
        }

    }
}

//How this is supposed the work:
//Step 1: See if button is being pressed
//Step 2: If button is being pressed, move turret in corresponding direction
//Step 3: Turret Continues moving while button pressed
//Step 4: When button stops being pressed, stop moving
//Step 5: Repeat for both buttons/directions
