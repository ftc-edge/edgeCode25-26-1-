package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp()
public class limelightTest extends OpMode {
    private Limelight3A limelight;
    private double rotateOffset = 0;
    private float angleToPoint = 9000;
    private Servo servo;
    private double servoPos = 0.5;

    private double servoPosConversion(double tx){
        double convertedTx = tx / angleToPoint;
        telemetry.addData("offset Correction:", convertedTx);
        return convertedTx;
    }

    private void servoTest(){
        if(gamepad1.a){
            servoPos = 1;
            telemetry.addData("buttonPressed", 1);
        }if(gamepad1.b){
            servoPos = 0;
        }if(gamepad1.x){
            servoPos = 0.5;
        }
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servo = hardwareMap.get(Servo.class, "turretServo");

        //telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        limelight.start();
        telemetry.addData("begin opmode", 1);
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();
        servoTest();
        servo.setPosition(servoPos);
        telemetry.addData("servo position: ", servoPos);
        telemetry.addData("Offset", rotateOffset);
        if (result != null) {
            if (result.isValid()) {
                //obtain offset
                Pose3D botpose = result.getBotpose();
                rotateOffset = -1 * result.getTx();
                telemetry.addData("tx", result.getTx());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("Botpose", botpose.toString());

                //move servo to counteract the offset

                    servoPos += servoPosConversion(rotateOffset);
            }
        }
    }
}
