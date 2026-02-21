package org.firstinspires.ftc.teamcode.tests;
import android.content.res.AssetManager;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.Drive;
import org.firstinspires.ftc.teamcode.yaiba.BODY;
import org.firstinspires.ftc.teamcode.yaiba.ModelInputMapper;
import org.firstinspires.ftc.teamcode.yaiba.YAIBAONNX;

import ai.onnxruntime.OrtException;


@Autonomous
public class modelInputTest extends OpMode {
    private YAIBAONNX yaiba;

    AssetManager assetManager;

    private static DcMotor leftFront;
    private static DcMotor leftBack;
    private static DcMotor rightFront;
    private static DcMotor rightBack;

    @Override
    public void init() {

        rightFront = hardwareMap.get(DcMotor.class, "FRmotor");
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        leftFront = hardwareMap.get(DcMotor.class, "FLmotor");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);

        rightBack = hardwareMap.get(DcMotor.class, "BRmotor");
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        leftBack = hardwareMap.get(DcMotor.class, "BLmotor");
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        
        try {
            // Load AI model
            yaiba = new YAIBAONNX(hardwareMap.appContext.getAssets());
            telemetry.addData("Status", "Model loaded successfully");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addData("Error", "Failed to load model: " + e.getMessage());
            telemetry.update();
            return;
        }
        android.content.Context context = hardwareMap.appContext;
        assetManager = context.getAssets();
    }

    public float[] buildObservations(){
        float[] obs = new float[9];

        //Add your observation date here!
        /*
        *Order is:
        * obs0 = -relative distance Y
        * obs1 = relative distance X
        * obs2 = sin of heading
        * obs3 = cos of heading
        * osb4 = sin of target
        * obs5 = cos of target
        * obs6 = 0 for no route point, 1 for route point
        * obs7 = relative distance X routepoint
        * obs8 = relative distance Y routepoint
         */

        return obs;
    }

    @Override
    public void loop() {

        float[] observations = buildObservations();
        float[] actions = null;
        try {
            actions = yaiba.predict(observations, assetManager);
        } catch (OrtException e) {
            throw new RuntimeException(e);
        }

        float strafe = -actions[0];
        float forward = actions[1];
        float rotation = actions[2];

        rightFront.setPower(forward + strafe + rotation);
        rightBack.setPower(forward - strafe + rotation);
        leftFront.setPower(-forward - strafe - rotation);
        leftBack.setPower(-forward + strafe - rotation);

    }
}
