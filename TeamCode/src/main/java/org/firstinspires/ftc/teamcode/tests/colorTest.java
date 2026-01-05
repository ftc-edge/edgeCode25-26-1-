package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.components.Color;
import com.qualcomm.hardware.rev.RevColorSensorV3;


import java.util.Arrays;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp()
public class colorTest extends OpMode {

    //RevColorSensorV3 color;

    NormalizedColorSensor color;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        //color.enableLed(false);
    }

    @Override
    public void loop() {
        NormalizedRGBA colors = color.getNormalizedColors();
        telemetry.addData("Hue", JavaUtil.colorToHue(colors.toColor()));
        telemetry.addData("Saturation", JavaUtil.colorToSaturation((colors.toColor())));
        telemetry.addData("Value", JavaUtil.colorToValue(colors.toColor()));
        telemetry.update();
    }
}