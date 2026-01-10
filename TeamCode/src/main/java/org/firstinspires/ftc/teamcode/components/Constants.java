package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double autoFinalStageMultiplier = 8f;
    public static float HOOD1 = 0.5f;
    public static float HOOD2 = 0.46f;
    public static float HOOD3 = 0.64f;
    public static float HOOD4 = 0.81f;
    public static float TURRET1 = 0.55f;
    public static float TURRET2 = 0.5f;
    public static float TURRET3 = 0.75f;
    public static float TURRET4 = 1f;
    public static float turretSpinSpeed = 0.5f;
    public static float initHeading = -90f;
    public static float distanceTolerance = 2.5f;
    public static float startX = 150f;
    public static float startY = 60f;
    public static float actionsIndex = 0;
    public static float reverseMultForward = 1;
    public static float reverseMultStrafe = -1;
    public static boolean positionRotation;
    public static float shootTargetY = 0f;
    public static float shootTargetX = 0f;
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

    public static int targetId = 20;
}
