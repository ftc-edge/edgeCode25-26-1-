package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.components.Color;

import java.util.Arrays;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
@TeleOp()
public class colorTest extends OpMode {

    Color colorSensor;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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