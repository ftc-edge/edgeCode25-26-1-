package org.firstinspires.ftc.teamcode.psuedo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class intakePsuedo extends OpMode{

    private RevColorSensorV3 color;

    float[] testArray = {0,0,0};
    int currentIndex = 0;

    private DcMotor spindex;
    private DcMotor intakeMotor;

    @Override
    public void init() {
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        spindex = hardwareMap.get(DcMotor.class, "spindex");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        color.enableLed(true);
        spindex.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        spindex.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        doIntake();
    }

    public void doIntake(){
        color.getNormalizedColors();
        if(gamepad1.right_bumper) {
            intakeMotor.setPower(1);
            if(color.green() > color.blue() && color.green() > color.red()) {
                testArray[currentIndex] = 1;
                currentIndex++;
                telemetry.addData("COLOR DETECTED:", "GREEN");
                float currentPosition = spindex.getCurrentPosition();
                spindex.setTargetPosition((int) (currentPosition + 720));
                spindex.setPower(1);
            }else if(color.blue() > color.green() && color.red() > color.green()) {
                testArray[currentIndex] = -1;
                currentIndex++;
                telemetry.addData("COLOR DETECTED:", "PURPLE");
                float currentPosition = spindex.getCurrentPosition();
                spindex.setTargetPosition((int) (currentPosition + 720));
                spindex.setPower(1);
            }
        }
    }
}
