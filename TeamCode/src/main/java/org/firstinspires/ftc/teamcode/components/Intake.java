package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Intake{

    public static float intakePower = 1f;
    public static float reversePower = -0.5f;
    private DcMotor intakeMotor;
    Color colorSensor;

    public boolean paused;


    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    public void togglePower(float power){
        if (intakeMotor.getPower() != 0) {
            this.setPower(0);
        } else {
            this.setPower(power);
        }
    }

    public void pause(float time, ElapsedTime timer , float power){
            if (timer.milliseconds() < time) {
                this.setPower(-0.25f);
            } else {
                this.setPower(power);
                paused = false;
            }
    }
    public double getPower(){
        return intakeMotor.getPower();
    }
    public void setPower(float power){
        intakeMotor.setPower(power);

        /*
        // Check if it actually has the ball
        if(colorSensor.getColor() == "None"){
            return;
        }

        // Now the ball is inside, check color
        if(colorSensor.getColor() == "Green") {
            currentArray[currentIndex] = 1;
            telemetry.addData("COLOR DETECTED:", "GREEN");

        }else if(colorSensor.getColor() == "Purple") {
            currentArray[currentIndex] = -1;
            telemetry.addData("COLOR DETECTED:", "PURPLE");
        }
        */

        // Spin indexer to free up space
    }

//    public void telemetry(){
//        telemetry.addData("Intake Power", intakeMotor.getPower());
//    }
}
