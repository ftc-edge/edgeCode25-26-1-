package org.firstinspires.ftc.teamcode.teleop;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.components.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@TeleOp
public class teleopDrive extends OpMode {
//    private Drive drive;
//    private Intake intake;
    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;
    private DcMotor intake;
    private DcMotor shoot1;
    private DcMotor shoot2;
    private DcMotor spindex;
    private boolean doIntake = false;
    private boolean doShoot = false;
    private boolean crossPress = false;
    private boolean circlePress = false;
    private boolean squarePress = false;
    private boolean trianglePress = false;

    private float shootPower;
    private float turretServo1Position = 0;
    private float turretServo2Position = 1;

    public final static float spindexRotation = 751.8f;

    private Servo turretServo1;
    private Servo turretServo2;
    private Servo hood;
    private GoBildaPinpointDriver odo;

    @Override
    public void init() {
        rightFront  = hardwareMap.get(DcMotor.class, "FRmotor");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftFront = hardwareMap.get(DcMotor.class, "FLmotor");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);

        rightBack = hardwareMap.get(DcMotor.class, "BRmotor");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack = hardwareMap.get(DcMotor.class, "BLmotor");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        shoot1 = hardwareMap.get(DcMotor.class, "shoot1");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setDirection(DcMotor.Direction.FORWARD);

        shoot2 = hardwareMap.get(DcMotor.class, "shoot2");
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setDirection(DcMotor.Direction.REVERSE);

        spindex = hardwareMap.get(DcMotor.class, "spindex");
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindex.setDirection(DcMotor.Direction.FORWARD);

        turretServo1 = hardwareMap.get(Servo.class, "turretServo1");
        turretServo2 = hardwareMap.get(Servo.class, "turretServo2");
        hood = hardwareMap.get(Servo.class, "hood");

        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.setOffsets(18.8, -13, DistanceUnit.CM);
        Pose2D startPose = new Pose2D(DistanceUnit.CM, 0, 0, AngleUnit.DEGREES, 0);
        odo.setPosition(startPose);
    }

    @Override
    public void loop() {
        double forward = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        rightFront.setPower(forward + strafe + pivot);
        rightBack.setPower(forward - strafe + pivot);
        leftFront.setPower(forward - strafe - pivot);
        leftBack.setPower(forward + strafe - pivot);

        shoot1.setPower(shootPower);
        shoot2.setPower(shootPower);

        if(turretServo1Position > 1){
            turretServo1Position = 1;
        }else if(turretServo1Position < 0){
            turretServo1Position = 0;
        }
        turretServo1.setPosition(turretServo1Position);
        turretServo2.setPosition(1 - turretServo1Position);

        gamepadManagement();

        doIntake();
    }

    private void doIntake(){
        if(doIntake){
            intake.setPower(1);
        }else {
            intake.setPower(-1);
        }
    }

    private void doShoot(){
        spindex.setTargetPosition((int) (spindex.getCurrentPosition() - spindexRotation));
    }

    private void gamepadManagement(){
        //cross
        if(gamepad1.crossWasPressed() && !crossPress){
            crossPress = true;
            if(doIntake){
                doIntake = false;
            }
            else{
                doIntake = true;
            }
        }
        if(gamepad1.crossWasReleased()){
            crossPress = false;
        }

        //square
        if(gamepad1.squareWasPressed() && !squarePress){
            squarePress = true;

        }
        if(gamepad1.squareWasReleased()){
            squarePress = false;
        }

        //circle
        if(gamepad1.circleWasPressed() && !circlePress){
            circlePress = true;
        }
        if(gamepad1.circleWasReleased()){
            circlePress = false;
        }

        //triangle
        if(gamepad1.triangleWasPressed() && !trianglePress){
            trianglePress = true;
            doShoot();
        }
        if(gamepad1.crossWasReleased()){
            trianglePress = false;
        }

        if(gamepad1.right_bumper){
            turretServo1Position -= 0.01F;
        }
        if(gamepad1.left_bumper){
            turretServo1Position += 0.01F;
        }
    }
}
