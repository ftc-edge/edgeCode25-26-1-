package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.TurretAutoAimODO;

@TeleOp
public class turretAutoAimOdoTest extends OpMode {
    TurretAutoAimODO turretAutoAimODO;
    Drive drive;

    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretAutoAimODO = new TurretAutoAimODO(hardwareMap, 0, 0.5, "teleop");
        drive = new Drive(hardwareMap);
    }

    public void loop(){
        turretAutoAimODO.runToAim(telemetry);

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        Drive.setPower(forward, strafe, pivot);

        telemetry.update();
    }
}
