package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Constants;

@TeleOp
public class maxtestingthingy extends OpMode {

    private DcMotor shoot1;
    private DcMotor shoot2;
    private DcMotor spindex;
    private DcMotor intake;
    //private Servo servo;

    private boolean rightPressed = false;
    private boolean leftPressed = false;
    private float servoPosition;

    @Override
    public void init() {
        shoot1 = hardwareMap.get(DcMotor.class, "shoot1");
        shoot1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot1.setDirection(DcMotor.Direction.REVERSE);

        shoot2 = hardwareMap.get(DcMotor.class, "shoot2");
        shoot2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoot2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shoot2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shoot2.setDirection(DcMotor.Direction.FORWARD);

        spindex = hardwareMap.get(DcMotor.class, "spindex");
        spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindex.setTargetPosition(0);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindex.setDirection(DcMotor.Direction.FORWARD);

        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setDirection(DcMotor.Direction.FORWARD);

        //servo = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void loop() {
        intake.setPower()
//        shoot1.setPower(0.75);
//        shoot2.setPower(0.75);

        //servo.setPosition(servoPosition);

        if(gamepad1.dpad_right && !rightPressed){
            spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindex.setTargetPosition((int) (spindex.getCurrentPosition() + ((float) 1 /3) * Constants.spindexRotation));
            spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindex.setPower(1);
            rightPressed = true;
        }
        if(gamepad1.dpadRightWasReleased()){
            rightPressed = false;
        }
        if(gamepad1.dpad_left && !leftPressed){
            spindex.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            spindex.setTargetPosition((int)(spindex.getCurrentPosition() - ((float) 1 /3) * Constants.spindexRotation));
            spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            spindex.setPower(1);
            leftPressed = true;
        }
        if(!gamepad1.dpad_left){
            leftPressed = false;
        }

        if(gamepad1.dpad_up){
            servoPosition += 0.002;
        }
        if(gamepad1.dpad_down){
            servoPosition -= 0.002;
        }

        telemetry.addData("spindex position", spindex.getCurrentPosition());
    }
}
