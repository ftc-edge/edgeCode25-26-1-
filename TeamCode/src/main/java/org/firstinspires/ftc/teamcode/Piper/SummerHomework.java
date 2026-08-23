package org.firstinspires.ftc.teamcode.Piper;

//21087 Velocity
// Bot notes:
// Moving; 4 Mechanum wheels
// Odometry, gets robot's coordinates and uses that to adjust the turret, pinpoint
// 3 rows of gecko wheels intake and pivoting wheel to change pressure applied
// linear sorting mechanism, pushes ball out of mechanism then puts it back in when want it
// mini servos to move turret, 270 degree shooting
//2 motor with gears to power flywheel
// servo to move hood position
// distance sensor to tell how many balls in robot `\(:/)/`
// 2 servos that pivot robot onto angle for end game

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;

// Moving:
    public class SummerHomework extends OpMode {
        DcMotor FL;
        DcMotor FR;
        DcMotor BL;
        DcMotor BR;

        Servo HoodServo1;
        Servo HoodServo2;
        Servo HoodServo3;

        float LeftStick_x;
        float LeftStick_y;
        float RightStick_x;

        DcMotor Intake1;
        DcMotor Intake2;
        DcMotor Intake3;

        float LeftTrigger;

        DcMotor flywheelMotor;

        Servo pivotServo1;
        Servo pivotServo2;

        GoBildaPinpointDriver pinpoint;

        DcMotor Slide;

        boolean Up;
        boolean A;
        boolean B;
        boolean Left;
        boolean Right;

        int Position = 1; //Based on starting Position: 1 is Blue Far, 2 is Blue Near, 3 is Red Far, 4 is Red Near

        public void init(){
            DcMotor FL = hardwareMap.get(DcMotor.class, "FL");
            DcMotor FR = hardwareMap.get(DcMotor.class, "FR");
            DcMotor BL = hardwareMap.get(DcMotor.class, "BL");
            DcMotor BR = hardwareMap.get(DcMotor.class, "BR");

            GoBildaPinpointDriver pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setOffsets(18,18, DistanceUnit.CM);
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            if (Position == 1 || Position == 2) {
                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }
            if (Position == 3 || Position == 4) {
                pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
            }

            pinpoint.resetPosAndIMU();
            if (Position == 1 || Position == 3){
                Pose2D startingPosition = new Pose2D(DistanceUnit.INCH, 48, 12, AngleUnit.DEGREES, 0);
            }
            if (Position == 2 || Position == 4) {
                Pose2D startingPosition = new Pose2D(DistanceUnit.INCH, 22, 122, AngleUnit.DEGREES, 135);
            }

            Servo HoodServo1 = hardwareMap.get(Servo.class, "HoodServo1");
            Servo HoodServo2 = hardwareMap.get(Servo.class, "HoodServo2");
            Servo HoodServo3 = hardwareMap.get(Servo.class, "HoodServo3");

            DcMotor Intake1 = hardwareMap.get(DcMotor.class, "Intake1");
            DcMotor Intake2 = hardwareMap.get(DcMotor.class, "Intake2");
            DcMotor Intake3 = hardwareMap.get(DcMotor.class, "Intake3");

            DcMotor flywheelMotor = hardwareMap.get(DcMotor.class, "flywheelMotor");

            Servo pivotServo1 = hardwareMap.get(Servo.class, "pivotServo1");
            Servo pivotServo2 = hardwareMap.get(Servo.class, "pivotServo2");

            DcMotor Slide = hardwareMap.get(DcMotor.class, "Slide");

            float LeftStick_x = gamepad1.left_stick_x;
            float LeftStick_y = gamepad1.left_stick_y;
            float RightStick_x = gamepad1.right_stick_x;
            float LeftTrigger = gamepad1.left_trigger;
            boolean A = gamepad1.a;
            boolean B = gamepad1.b;
            boolean Up = gamepad1.dpad_up;
            boolean Left = gamepad1.dpad_left;
            boolean right = gamepad1.dpad_right;
        }

        public void Drive(){
            FL.setPower(LeftStick_x + LeftStick_y + RightStick_x);
            FR.setPower(-LeftStick_x + LeftStick_y - RightStick_x);
            BL.setPower(LeftStick_x + LeftStick_y + RightStick_x);
            BR.setPower(-LeftStick_x + LeftStick_y - RightStick_x);
        }

        public void Aim(){
            Pose2D pos = pinpoint.getPosition();
            double x = pinpoint.getPosX(DistanceUnit.INCH);
            double y = pinpoint.getPosX(DistanceUnit.INCH);
            double angle = -1*Math.tan(x/(144-y));
            HoodServo1.setPosition(angle);
            HoodServo2.setPosition(angle);
            double n;
            double v;
            double f;
            double w;
            n = Math.hypot(132-y,12-x);
            w = Math.sqrt(34848);
            v = n / w;
            f = 45*v;
            HoodServo3.setPosition(f);
        }

        public void Sort(){
            if(Left){
                Slide.setPower((1));
            }
            if(Right){
                Slide.setPower(-1);
            }
        }

        public void Intake(){
            double power;
            power = 0;
            if(B){
                power = 0.75;
            }
            if(A){
                power = 0;
            }
            Intake1.setPower(power);
            Intake2.setPower(power);
            Intake3.setPower(power);
        }

        public void Outtake(){
            if (LeftTrigger > 0.3){
                flywheelMotor.setPower(0.75);
            }
        }

        public void Pivot(){
            if (Up){
                pivotServo1.setPosition(90);
                pivotServo2.setPosition(90);
            }
        }

        public void loop(){
            Pose2D pos = pinpoint.getPosition();
            Drive();
            Aim();
            Sort();
            Intake();
            Outtake();
            Pivot();
            pinpoint.update();
        }
        
    }