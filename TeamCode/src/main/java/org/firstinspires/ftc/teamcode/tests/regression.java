package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Turret;

import java.util.HashMap;
@TeleOp
public class regression extends OpMode {
    Turret turret;
    Hood hood;

    private float spinSpeed;
    private float hoodPos;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        hood = new Hood(hardwareMap);


    }

    @Override
    public void loop() {
        hood.setPosition(hoodPos);
        turret.setTargetRPM(spinSpeed);

        telemetry.addData("flywheel", spinSpeed);
        telemetry.addData("hood", hoodPos);
        telemetry.update();
        if(gamepad1.right_trigger > 0.5){
            spinSpeed += 0.001;
        }

        if(gamepad1.left_trigger > 0.5){
            spinSpeed -= 0.001;
        }

        if(gamepad1.right_bumper){
            hoodPos += 0.0005;
        }
        if(gamepad1.left_bumper){
            hoodPos -= 0.0005;
        }
    }


}
