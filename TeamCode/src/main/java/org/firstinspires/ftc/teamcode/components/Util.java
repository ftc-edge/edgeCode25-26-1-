package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.components.AutoConstants;

public class Util {
    public static int getTargetId(){
        String color = AutoConstants.allianceColor.toLowerCase().replace(" ", "");
        if (color.equals("red")){
            return 24;
        }
        return 20;
    }
}
