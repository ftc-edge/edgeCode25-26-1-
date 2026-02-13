package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretSpin;

@TeleOp
public class autoAimHelper extends OpMode {
    TurretSpin turretSpin;

    private float turretPos = 0;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        turretSpin = new TurretSpin(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.dpad_up){
            turretSpin.spinRightCR(0.4);
        }
        if (gamepad1.dpad_down){
            turretSpin.spinLeftCR(0.4);
        }

        turretSpin.autoAim();
        telemetry.addData("position", turretSpin.getCurrentPosition());
        telemetry.addData("distToAprilTag", turretSpin.distToAprilTag);
        telemetry.update();
    }

}
