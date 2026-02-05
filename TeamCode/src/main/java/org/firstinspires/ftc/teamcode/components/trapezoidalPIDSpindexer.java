package org.firstinspires.ftc.teamcode.components;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.acmerobotics.dashboard.config.Config;

@Config
public class trapezoidalPIDSpindexer {
    public static float spindexRotation = 385; // originally 538
    public static float Kp = 0.01f;
    public static float Ki = 0.03f;
    public static float Kd = 0.1f;
    public static float Kf = -0.00002f;
    public static double tolerance = 1;

    private PIDFController pidf;

    // Constants - Tune these for your specific motor/spindexer
    // State Variables
    public  double relativeTarget, finalTarget;
    private boolean isRunning = false;

    // Call this once to start a 120-degree move
    public void spinNumTurns(double numTurns) {
        //relativeTarget = degrees * TICKS_PER_DEGREE;
        relativeTarget = (spindexRotation / 3) * numTurns;
        finalTarget += relativeTarget;
    }

    public void spin(int spinAmount){
        finalTarget += spinAmount;
    }

    public void stop(double currentPos){
        finalTarget = currentPos;
        update(currentPos);
    }

    // Call this every single frame in your main loop
    public double update(double currentPos) {
        if (Math.abs(currentPos - finalTarget) > tolerance) isRunning = true;
        if (!isRunning) return 0;

        pidf = new PIDFController(Kp, Ki, Kd, Kf);
        pidf.setTolerance(tolerance);

        pidf.setSetPoint(finalTarget);

        if (Math.abs(currentPos - finalTarget) < tolerance) isRunning = false;

        // Calculate PID output based on the "Moving Target"

        return pidf.calculate(currentPos, finalTarget);
    }
}
