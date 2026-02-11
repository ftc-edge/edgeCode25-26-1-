package org.firstinspires.ftc.teamcode.components;
import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoConstants {
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
    public static float initHeading = -90f;
    public static float distanceTolerance = 2.5f;
    public static float startX = 60f;
    public static float startY = 150f;
    public static float actionsIndex = 0;
    public static float reverseMultForward = 1;
    public static float reverseMultStrafe = 1;
    public static boolean positionRotation;
    public static float shootTargetY = -60f;
    public static float shootTargetX = 60f;
    public static float firstIntakePrepY = -60f;
    public static float firstIntakePrepX = -30f;
    public static float firstIntakeY = -120f;
    public static float firstIntakeX = -30f;
    public static float secondIntakePrepY = -60f;
    public static float secondIntakePrepX = 30f;
    public static float secondIntakeY = -120f;
    public static float secondIntakeX = 30f;
    public static float thirdIntakePrepY = -60f;
    public static float thirdIntakePrepX = 90f;
    public static float thirdIntakeY = 120f;
    public static float thirdIntakeX = 90f;
    public static float humanPlayerPrepY = 90f;
    public static float humanPlayerPrepX = 150f;
    public static float humanPlayerY = 150f;
    public static float humanPlayerX = 150f;

    public static enum autoMode{
        blueFront,
        blueBack,
        redFront,
        redBack;
    }

    public static autoMode currentAuto;
}
