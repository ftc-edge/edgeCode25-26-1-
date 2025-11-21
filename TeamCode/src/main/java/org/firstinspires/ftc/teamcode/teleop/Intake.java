package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.dashboard.canvas.Canvas;

@TeleOp
public class Intake extends OpMode{

    private DcMotor spindex;
    private DcMotor intakeMotor;
    Color colorSensor;

    public static int[] spindexPositions = {0,720,1440};

    float[] currentArray = {0,0,0};
    int currentIndex = 0;

    private void spinSpindexToPos(int position, float power){
        spindex.setPower(power);
        spindex.setTargetPosition(spindexPositions[position]);
        currentIndex = position;
    }

    private boolean spinSpindexToNextFree(float power){
        // Return true if found a free position, false if else
        int finalPos = -1;
        for(int i = 0; i < spindexPositions.length; i++){
            int index = (currentIndex + i) % spindexPositions.length;
            if(currentArray[index] == 0){
                finalPos = index;
                break;
            }
        }
        if(finalPos == -1){
            return false;
        }
        spinSpindexToPos(finalPos, power);
        return true;
    }

    private boolean spinSpindexToColor(int color, float power){
        // Return true if found a free position, false if else
        int finalPos = -1;
        for(int i = 0; i < spindexPositions.length; i++){
            int index = (currentIndex + i) % spindexPositions.length;
            if(currentArray[index] == color){
                finalPos = index;
                break;
            }
        }
        if(finalPos == -1){
            return false;
        }

        spinSpindexToPos(finalPos, power);
        return true;
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        spindex = hardwareMap.get(DcMotor.class, "spindex");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        if(gamepad1.right_bumper) {
            doIntake();
        }

        // Telemetry
        telemetry.addData("Spindexer Array", currentArray);
        telemetry.addData("Current Position", currentIndex);
    }

    public void doIntake(){
        intakeMotor.setPower(1);

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

        // Spin indexer to free up space
        spinSpindexToNextFree(.5f);

    }
}
