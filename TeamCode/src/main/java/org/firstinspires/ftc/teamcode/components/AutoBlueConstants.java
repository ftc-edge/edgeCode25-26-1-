package org.firstinspires.ftc.teamcode.components;
import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoBlueConstants {
    public static double shootScaled1 = 160;
    public static double shootScaled2 = 175;
    public static double shootScaled3 = 175;
    public static String allianceColor = "Red";
    public static String startingPosition = "Front";
    public static double telemetryScale = 60;
    public static double driveForwardMult = 1;
    public static double driveStrafeMult = -1;
    public static double driveRotationMult = 1;
    public static double MODEL_POS_SCALE = 0.000625;
    public static int beforeShootDelayMS = 500;

    public static enum autoMode{
        blueFront,
        blueBack,
        redFront,
        redBack;
    }

    public static autoMode currentAuto;

    public static double intakePrepY = -0.33;
    public static double intakeY = -0.9;
    public static double intake1PrepX = 0.125;
    public static double intake2PrepX = 0.48;
    public static double intake3PrepX = -0.20;
    public static double gatePushPrepX = 0.075;
    public static double gatePushPrepY = -0.5;
    public static double gatePushY = -0.85;
    public static double gateHeading = -2;
    public static double gatePickupX = 0.2425;
    public static double gatePickupY = -0.915;

    public static double shootX = -0.25;
    public static double shootY = -0.25;

    public static double gateTime = 3;
    public static double pushTime = 0.25;
    public static double intakeTime = 1.5;
}
