package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoRedConstants {
    public static enum autoMode {
        blueFront,
        blueBack,
        redFront,
        redBack;
    }

    public static double MODEL_POS_SCALE = 0.000625;
    public static int beforeShootDelayMS = 500;
    public static double driveForwardMult = 1;
    public static double driveMult = 1;
    public static double driveRotationMult = 1;
    public static double driveStrafeMult = -1;
    public static double gateHeading = 2;
    public static double gatePickupX = 0.2425;
    public static double gatePickupY = 0.975;
    public static double gatePushPrepX = 0.05;
    public static double gatePushPrepY = 0.5;
    public static double gatePushY = 0.85;
    public static double gateTime = 1.75;
    public static double hoodPos = 0.775;
    public static double intake1PrepX = 0.1375;
    public static double intake2PrepX = 0.48;
    public static double intake3PrepX = -0.2;
    public static double intake3y = 0.825;
    public static double intakePrepY = 0.45;
    public static double intakeTime = 1.5;
    public static double intakeY = 0.975;
    public static double pushTime = 0.75;
    public static double reversePower = -0.6;
    public static double reverseTime = 0.5;
    public static double shootScaled1 = 160;
    public static double shootScaled2 = 175;
    public static double shootScaled3 = 175;
    public static float shootSpeed = 130;
    public static double shootX = -0.25;
    public static double shootY = 0.25;
    public static String startingPosition = "Front";
    public static double telemetryScale = 60;

}