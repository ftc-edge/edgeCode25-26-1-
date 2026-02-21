package org.firstinspires.ftc.teamcode.components;

public class Util {
    public static int getTargetId(){
        String color = Constants.allianceColor.toLowerCase().replace(" ", "");
        if (color.equals("red")){
            return 24;
        }
        return 20;
    }

    public static String getColor(){
        String color = Constants.allianceColor.toLowerCase().replace(" ", "");
        return color;
    }
}
