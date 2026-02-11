package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;

@Config
public class SpindexProfile {
    // Hardware Constraints
    double maxVelocity;     // Max ticks/sec (e.g., 2500)
    double maxAcceleration; // Max ticks/sec^2 (e.g., 5000)

    // Profile Timings
    double timeToRamp;      // Seconds to reach max speed
    double timeToCruise;    // Seconds spent at max speed
    double totalProfileTime;

    // Profile Distances
    double distToRamp;      // Ticks covered during acceleration
    double totalDistance;   // Total ticks to move

    public void planProfile(double distanceToMove) {
        this.totalDistance = distanceToMove;

        // 1. How long does it take to reach max velocity? (v = a * t)
        timeToRamp = maxVelocity / maxAcceleration;

        // 2. How far do we travel during that ramp? (d = 1/2 * a * t^2)
        distToRamp = 0.5 * maxAcceleration * Math.pow(timeToRamp, 2);

        // 3. Handle "Short Trips" (Triangular Profile)
        // If the move is too short to reach max speed, we must adjust
        if (distToRamp * 2 > totalDistance) {
            timeToRamp = Math.sqrt(totalDistance / maxAcceleration);
            distToRamp = 0.5 * totalDistance; // Ramp up half, ramp down half
            timeToCruise = 0;
        } else {
            // "Long Trip" (Trapezoidal Profile)
            double cruiseDistance = totalDistance - (distToRamp * 2);
            timeToCruise = cruiseDistance / maxVelocity;
        }

        totalProfileTime = (timeToRamp * 2) + timeToCruise;
    }}
