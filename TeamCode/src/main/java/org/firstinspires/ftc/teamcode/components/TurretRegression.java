package org.firstinspires.ftc.teamcode.components;

/*
49	0	108.63
60	0.145	120.11
70	0.255	128.62
80	0.39	135.32
90	0.425	135.54
100	0.425	135.54
110	0.425	135.54
120	0.445	133.84
130	0.445	133.84
140	0.5	127.06
150	0.5	127.06
160	0.5	133.796
170	0.5	133.796
180	0.5	133.02
190	0.5	133.12
200	0.445	133.12

*/


public class TurretRegression {
    int[] distances = {49,60,70,80,90,100,110,120,130,140,150,160,170,180,190,200};
    double[] hoodPosition = {0,0.145,0.255,0.39,0.425,0.425,0.425,0.445,0.445,0.5,0.5,0.5,0.5,0.5,0.5,0.445};
    double[] turretRPM = {108.63,120.11,128.62,135.32,135.54,135.54,135.54,133.84,133.84,127.06,127.06,133.796,133.796,133.02,133.12,133.12};
    
    public double getHoodPosition(float distance){
        if (distance <= distances[0]){
            return hoodPosition[0];
        }
        if (distance >= distances[distances.length - 1]){
            return hoodPosition[distances.length - 1];
        }
        for (int i = 1; i < distances.length; i++ ){
            if (distance < distances[i]){
                // found the number: it is in between i-1 and i
                // find the line that interpolates between these two points
                
                double slope = (hoodPosition[i] - hoodPosition[i-1]) / (distances[i] - distances[i-1]);
                // y = m(x-x1)+y1
                double estimatedHoodPosition = slope * (distance - distances[i]) + hoodPosition[i];
                return estimatedHoodPosition;
            }
        }
        return -1;
    }

    public double getTurretRPM(float distance){
        if (distance <= distances[0]){
            return turretRPM[0];
        }
        if (distance >= distances[distances.length - 1]){
            return turretRPM[distances.length - 1];
        }
        for (int i = 1; i < distances.length; i++ ){
            if (distance < distances[i]){
                // found the number: it is in between i-1 and i
                // find the line that interpolates between these two points
                
                double slope = (turretRPM[i] - turretRPM[i-1]) / (distances[i] - distances[i-1]);
                // y = m(x-x1)+y1
                double estimatedTurretRPM = slope * (distance - distances[i]) + turretRPM[i];
                return estimatedTurretRPM;
            }
        }
        return -1;
    }
}
