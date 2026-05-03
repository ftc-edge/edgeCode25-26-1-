package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Color {
    public static int saturationLimit = 3;
    public static int brightnessLimit = 33;
    public static int greenHueLowerLimit = 120;
    public static int greenHueUpperLimit = 225;
    public static int purpleHueLowerLimit = 225;
    public static int purpleHueUpperLimit = 340;

    ColorSamplerUtil colorSampler;

    public Color(HardwareMap hardwareMap) {
        colorSampler = new ColorSamplerUtil(hardwareMap, "Webcam 1", 6);
    }

    public String getColor() {
        ColorSamplerUtil.Sample s = colorSampler.getSample();
        float hue = s.frameMeanH;
        float sat = s.frameMeanS;
        float val = s.frameMeanL;

        if (sat <= saturationLimit) {
            return "NONE";
        }

        if (val <= brightnessLimit) {
            return "NONE";
        }

        if (hue >= greenHueLowerLimit && hue <= greenHueUpperLimit) {
            return "GREEN";
        }
        if (hue >= purpleHueLowerLimit && hue <= purpleHueUpperLimit) {
            return "PURPLE";
        }

        return "NULL";
    }

    public float[] getHSL() {
        ColorSamplerUtil.Sample s = colorSampler.getSample();
        return new float[]{s.frameMeanH, s.frameMeanS, s.frameMeanL};
    }

}
