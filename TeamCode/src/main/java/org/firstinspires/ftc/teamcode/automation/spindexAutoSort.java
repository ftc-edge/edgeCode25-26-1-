package org.firstinspires.ftc.teamcode.automation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.components.Spindex;

public class spindexAutoSort{
    public boolean firstGreen;

    public Spindex spindex;

    public enum targetMotif {
        GPP,
        PGP,
        PPG;
    }

    public targetMotif target;

    public spindexAutoSort(HardwareMap hardwareMap){
        spindex = new Spindex(hardwareMap);
    }

    public void sortNShoot(int[] layout, targetMotif target) {
        int greenLocation = -1;
        for (int i = 0; i < layout.length; i++) {
            if (layout[i] == 1) {
                greenLocation = i;
                i = layout.length;
            }
        }

        if (target == targetMotif.GPP){
            if(greenLocation == 1){
                spindex.spinTurns(2);
            }else if(greenLocation == 2){
                spindex.spinTurns(1);
            }
        }else if(target == targetMotif.PGP){
            if(greenLocation == 0 || greenLocation == 1){
                spindex.spinTurns(1);
            }
        }else{
            if(greenLocation == 0 || greenLocation == 2){
                spindex.spinTurns(2);
            }
        }
    }
}
