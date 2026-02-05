package org.firstinspires.ftc.teamcode.components;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double autoAimLoseDecayMultiplier = 0.98;
    public static double autoAimLoseMultiplier = 0.9;
    public static double limelightKP = 0.01;
    public static double limelightKD = 0.009;
    public static String defaultMotif = "None Detected";
    public static float intakeReverseTime = 500;
    public static int autoSortDelayMs = 0;
    public static float turretAdjustSpeed = 0.5f;
    public static float desiredHeading = 180f;
    public static float HOOD1 = 0f;
    public static float initHeading = -90f;
    public static float startX = 60f;
    public static float startY = 150f;
    
    public static float imuKp = 0.6f;
}
