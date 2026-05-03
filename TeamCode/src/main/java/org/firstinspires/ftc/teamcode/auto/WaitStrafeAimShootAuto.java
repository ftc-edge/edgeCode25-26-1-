package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.components.Color;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.components.Hood;
import org.firstinspires.ftc.teamcode.components.SpindexPID;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.TurretAutoAimODO;

@Autonomous(name = "Wait 23 Strafe Aim Shoot", group = "Simple")
@Config
public class WaitStrafeAimShootAuto extends LinearOpMode {

    public static double WAIT_SECONDS = 23.0;
    public static double STRAFE_POWER = 0.50;
    public static long STRAFE_MS = 700;
    public static double AIM_SECONDS = 1.2;
    public static double MAX_SHOOT_SECONDS = 6.0;

    public static double START_X = -0.944;
    public static double START_Y = -0.66;

    public static double SHOOT_SPEED = Constants.shootSpeed2;
    public static float HOOD_POS = Constants.HOOD2;

    @Override
    public void runOpMode() {
        Drive drive = new Drive(hardwareMap);
        SpindexPID pid = new SpindexPID(hardwareMap);
        Color color = new Color(hardwareMap);
        Turret turret = new Turret(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        TurretAutoAimODO aim = new TurretAutoAimODO(hardwareMap, 0, 0, "special");

        telemetry.addLine("Ready: wait 23s, strafe, aim, then shoot.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        hood.setPosition(HOOD_POS);
        turret.setTargetRPM(SHOOT_SPEED);

        ElapsedTime stageTimer = new ElapsedTime();

        // Stage 1: Wait
        stageTimer.reset();
        while (opModeIsActive() && !isStopRequested() && stageTimer.seconds() < WAIT_SECONDS) {
            updateAimingAndShooter(pid, aim, turret);
            Drive.setPower(0, 0, 0);
            telemetry.addData("Stage", "Waiting");
            telemetry.addData("Time Left", WAIT_SECONDS - stageTimer.seconds());
            telemetry.update();
        }

        // Stage 2: Strafe left (same direction style as moveAuto)
        stageTimer.reset();
        while (opModeIsActive() && !isStopRequested() && stageTimer.milliseconds() < STRAFE_MS) {
            updateAimingAndShooter(pid, aim, turret);
            Drive.setPower(0.0, STRAFE_POWER, 0.0);
            telemetry.addData("Stage", "Strafing");
            telemetry.update();
        }
        Drive.setPower(0.0, 0.0, 0.0);

        // Stage 3: Aim before shooting
        stageTimer.reset();
        while (opModeIsActive() && !isStopRequested() && stageTimer.seconds() < AIM_SECONDS) {
            updateAimingAndShooter(pid, aim, turret);
            telemetry.addData("Stage", "Aiming");
            telemetry.update();
        }

        // Stage 4: Shoot using same SpindexPID pattern as teleop/YAIBA
        boolean adjusted = false;
        boolean startedShoot = false;
        stageTimer.reset();

        while (opModeIsActive() && !isStopRequested() && stageTimer.seconds() < MAX_SHOOT_SECONDS) {
            updateAimingAndShooter(pid, aim, turret);

            if (!adjusted) {
                adjusted = pid.shootConsecutiveAdjust();
            } else if (!startedShoot && turret.atTarget() && pid.isAtTarget()) {
                pid.startShootConsecutive();
                startedShoot = true;
            }

            pid.shootConsecutive(color);

            telemetry.addData("Stage", "Shooting");
            telemetry.addData("Adjusted", adjusted);
            telemetry.addData("Started", startedShoot);
            telemetry.addData("SpindexShooting", pid.shooting);
            telemetry.update();

            if (startedShoot && !pid.shooting && pid.isAtTarget()) {
                break;
            }
        }

        Drive.setPower(0.0, 0.0, 0.0);
        pid.stop();
        turret.stop();
    }

    private void updateAimingAndShooter(SpindexPID pid, TurretAutoAimODO aim, Turret turret) {
        pid.update();
        aim.runToAim(telemetry);
        turret.loop();
    }
}
