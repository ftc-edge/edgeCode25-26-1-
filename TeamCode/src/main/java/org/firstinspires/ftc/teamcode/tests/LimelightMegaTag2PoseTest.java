package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;

@TeleOp(name = "Limelight MT2 Pose Test", group = "Test")
public class LimelightMegaTag2PoseTest extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Ready for MegaTag2 pose test");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double yawDeg = -180; // Always assume the limelight is pointing straight on
            boolean orientationUpdated = limelight.updateRobotOrientation(yawDeg);

            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                Pose3D botposeMt2 = result.getBotpose_MT2();
                telemetry.addData("Botpose_MT2", botposeMt2.toString());
                telemetry.addData("BotposeTagCount", result.getBotposeTagCount());
                telemetry.addData("BotposeAvgDist", result.getBotposeAvgDist());
                telemetry.addData("Staleness(ms)", result.getStaleness());
            } else {
                telemetry.addData("Limelight", "No valid result");
            }

            telemetry.addData("Yaw(deg)", yawDeg);
            telemetry.addData("OrientationUpdated", orientationUpdated);
            telemetry.update();
        }

        limelight.stop();
    }
}
