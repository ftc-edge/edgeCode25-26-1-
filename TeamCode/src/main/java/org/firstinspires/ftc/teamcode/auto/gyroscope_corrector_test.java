package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.responses.ImuAngularVelocityResponse;
import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.components.Constants;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;

public class gyroscope_corrector_test extends OpMode {
    public float gyroscopePosition;
    public float correctorPosition;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Gyroscope gyroscope;
    private GoBildaPinpointDriver gyroscope_corrector;
    private Constants constants;
    public float kp;


    @Override
    public void init() {
        correctorPosition = (float) gyroscope_corrector.getHeading(AngleUnit.DEGREES);
        gyroscopePosition = 90;
        kp = constants.imuKp;
    }

    @Override
    public void loop() {
        telemetry.addData("current position", gyroscope_corrector.getHeading(AngleUnit.DEGREES));
        telemetry.update();
        while (gyroscopePosition != correctorPosition) {
            frontLeft.setPower(1);
            frontRight.setPower(-1);
            backLeft.setPower(1);
            backRight.setPower(-1);

            float correctorPosition1 = (float) gyroscope_corrector.getHeading(AngleUnit.DEGREES);
            if (correctorPosition1 - correctorPosition > 360) {
                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
            }
        }
    }
}

// Gameplan! :
// decide target heading
// Add gyroscope corrector
// Step 1: Collect data on what the current heading is
// Step 2: Turn until gyroscope matches actual heading
