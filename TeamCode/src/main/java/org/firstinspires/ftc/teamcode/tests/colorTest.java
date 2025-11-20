package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.components.Color;

import java.util.Arrays;

@TeleOp()
public class colorTest extends OpMode {

    Color colorSensor;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        telemetry.addData("rgbDetected:", Arrays.toString(colorSensor.getRGB()));
        if(colorSensor.getColor() == "None"){
            telemetry.addData("color:", "None");
        }
        // Now the ball is inside, check color
        if(colorSensor.getColor() == "Green") {
            telemetry.addData("color:", "Green");

        }else if(colorSensor.getColor() == "Purple") {
            telemetry.addData("color:", "Purple");
        }
        telemetry.update();
    }
}