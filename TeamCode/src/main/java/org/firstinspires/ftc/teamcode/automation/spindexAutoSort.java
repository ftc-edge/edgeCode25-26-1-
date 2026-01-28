package org.firstinspires.ftc.teamcode.automation;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Spindex;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

public class spindexAutoSort{
    public boolean firstGreen;

    public Spindex spindex;

    public enum targetMotif {
        GPP,
        PGP,
        PPG;
    }

    public targetMotif target;

    Telemetry telemetry;

    public spindexAutoSort(HardwareMap hardwareMap, Telemetry telemetry){
        spindex = new Spindex(hardwareMap);
        this.telemetry = telemetry;
    }
    //layout: int [3]
    //1=green
    //-1=purple

    public void sortNShoot(int[] layout, targetMotif target, int currentPosition) {

        if(layout == null || layout.length != 3){
            telemetry.addLine("ERROR");
            telemetry.update();
            return;
        }

        int grnCount = 0;
        int purCount = 0;
        int greenLocation = -1;

        for (int i = 0; i < 3; i++) {
            if (layout[i] == 1) {
                grnCount++;
                greenLocation = i;
            }
            if (layout[i] == -1){
                purCount ++;
            }
        }
//            if (layout[i] == -1) {
//                purCount++;
//            }
//        }
        if(grnCount + purCount != 3){
            telemetry.addData("Error","Expect 3 Balls");
            telemetry.update();
        }
        if (grnCount != 1) {
            telemetry.addData("Error","Expect 1 Green",grnCount);
            telemetry.update();
            return;
        }

        telemetry.addLine("Auto Sort Telemetry:");
        telemetry.addData("green Location", greenLocation);
        telemetry.addData("targetMotif", target);


        if (target == targetMotif.GPP) {
            if (greenLocation == 1) {
                spindex.spinTurns(2);
            } else if (greenLocation == 2) {
                spindex.spinTurns(1);
            }
        } else if (target == targetMotif.PGP) {
            if (greenLocation == 0) {
                spindex.spinTurns(1);
            } else if (greenLocation == 2) {
                spindex.spinTurns(2);
            }
        } else {
            if (greenLocation == 0) {
                spindex.spinTurns(2);
            } else if (greenLocation == 1) {
                spindex.spinTurns(1);
            }
        }
        //could also do
        int turns = 0;
        switch(target){
            case GPP:
                if (greenLocation==1) turns = 2;
                else if (greenLocation==2) turns = 1;
                break;

                //use current pos - green pos not this logic bc if current pos = 1 index #1 is in front etc
            // if u want index one to be in the front, itd be (1- current position +3) mod3
            //currentPosition = (currentPosition + 1 ) % 3;

            case PGP:
                if (greenLocation==0) turns = 1;
                else if (greenLocation==2) turns = 2;
                break;

            case PPG:
                if (greenLocation==0) turns = 2;
                else if (greenLocation==1) turns = 1;
                break;

                }
        telemetry.addData("Sprindex Turns", turns);
        telemetry.update()
                if(turns>0) {
                    spindex.spinTurn ns(turns);
                }
            }

                }
            {

                }
        }

    }
}
