package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.Range;

@Config
public class TurretSpin {
    CRServo leftServo;
    CRServo rightServo;

    DcMotorEx servoEncoder;

    public Limelight3A limelight;

    public double lastError;
    public double prevErrorSign;

    double lastAutoAimPower = 0;

    public double distToAprilTag = 1;

    public static double targetHeight = .4;

    public LLResult result;

    public String detectedMotif = Constants.defaultMotif;

    //    public static float spinPower = 0.2f;

    public TurretSpin(HardwareMap hardwareMap) {
        leftServo = hardwareMap.get(CRServo.class, "leftServo");
        rightServo = hardwareMap.get(CRServo.class, "rightServo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        servoEncoder = hardwareMap.get(DcMotorEx.class, "BLmotor");

        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void autoAim(){
        result = limelight.getLatestResult();

        if (result == null || !result.isValid()){
            spinRightCR(lastAutoAimPower * Constants.autoAimLoseMultiplier);
            lastAutoAimPower *= Constants.autoAimLoseDecayMultiplier;
            return;
        }

        if (result.getFiducialResults().get(0).getFiducialId() != Util.getTargetId()){
            if(result.getFiducialResults().get(0).getFiducialId() == 21){
                detectedMotif = "GPP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 22){
                detectedMotif = "PGP";
            }else if(result.getFiducialResults().get(0).getFiducialId() == 23){
                detectedMotif = "PPG";
            }
            spinRightCR(lastAutoAimPower * Constants.autoAimLoseMultiplier);
            lastAutoAimPower *= Constants.autoAimLoseDecayMultiplier;
            return;
        }

        // Detected a target
        double tx = result.getTx();
        double error = tx; //u want tx=0

        double deadband = 1.0; //degrees

        double derivative = error - lastError;
        float power = (float) (Constants.limelightKP * error + Constants.limelightKD * derivative);

        lastError = error;


        if (Math.abs(error) < deadband) {
            power = 0;
        } else {
            power = (float) Range.clip(power, -0.75, 0.75);
        }

        lastAutoAimPower = power;
//        spinRightCR(power);

        distToAprilTag = Math.sqrt(Math.pow(result.getBotposeAvgDist(), 2) - Math.pow(targetHeight, 2));
        distToAprilTag *= Constants.regressionScaling;
    }

    public double getDistToAprilTag(){
        return distToAprilTag;
    }

    public double getCurrentPosition(){
        return servoEncoder.getCurrentPosition();
    }

    public void spinRightCR(double spinPower){
        leftServo.setPower(spinPower);
        rightServo.setPower(spinPower);
    }

    public void spinLeftCR(double spinPower){
        leftServo.setPower(-spinPower);
        rightServo.setPower(-spinPower);
    }


    public void spinToPos(double target){

    }
}
