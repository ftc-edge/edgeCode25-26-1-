package org.firstinspires.ftc.teamcode.tests;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.sun.tools.javac.util.Context;
import org.firstinspires.ftc.teamcode.tests.SensorGoBildaPinpointExample;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.yaiba.BODY;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.io.IOException;
@TeleOp
public class yaibaTest extends LinearOpMode{
        private BODY yaiba;
    @Override public void runOpMode() {
        try {
            // Make sure BODY constructor is the minimal one that does NOT run warmUp.
            yaiba = new BODY(hardwareMap.appContext);
            telemetry.addData("YAIBA", "Loaded OK");
            telemetry.update();
        } catch (IOException e) {
            telemetry.addData("YAIBA loadErr", e.getMessage());
            telemetry.addData("stack", android.util.Log.getStackTraceString(e));
            telemetry.update();
        }
        // stay alive so we can inspect logs
        waitForStart();
        while (opModeIsActive()) idle();
    }
    }
