package org.firstinspires.ftc.teamcode.automation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Spindex;

public class SpindexAutoSort {
    public boolean firstGreen;

    public Spindex spindex;

    public enum targetMotif {
        GPP,
        PGP,
        PPG;

    }

    public targetMotif target;

    Telemetry telemetry;

    public SpindexAutoSort(HardwareMap hardwareMap, Telemetry telemetry) {
        spindex = new Spindex(hardwareMap);
        this.telemetry = telemetry;
    }
    //layout: int [3]
    //1=green
    //-1=purple
    //currentPosition = index currently in front

    public int sortNShoot(int[] layout, String motif, int currentPosition) {

        if (layout == null || layout.length != 3) {
            telemetry.addLine("ERROR: Invalid layout");
            telemetry.update();
            return currentPosition;
        }

        int grnCount = 0;
        int purCount = 0;
        int greenLocation = -1;

        for (int i = 0; i < 3; i++) {
            if (layout[i] == 1) {
                grnCount++;
                greenLocation = i;
            } else if (layout[i] == -1) {
                purCount++;
            }
        }

        if (grnCount != 1 || grnCount + purCount != 3) {
            telemetry.addData("Error", "Not enough balls");
            telemetry.update();
            return currentPosition;
        }
        int wantedFrontIndex = 0;
        //which index @ front?


        switch (motif) {
            case "GPP":
                wantedFrontIndex = ((greenLocation)+3) % 3;
                break;
// incorporate code abt grnlocation
            //use current pos - green pos not this logic bc if current pos = 1 index #1 is in front etc
            // if u want index one to be in the front, itd be (1- current position +3) mod3
            //currentPosition = (currentPosition + 1 ) % 3;

            case "PGP":
                //wantedFrontIndex = (greenLocation + 1) % 3;
                //wantedFrontIndex = 1;
                wantedFrontIndex = ((1 - greenLocation) + 3) % 3;
                // set to greenlocation - 1
                break;

            case "PPG":
                //wantedFrontIndex = greenLocation;
                wantedFrontIndex = (2-greenLocation);
                // set to greenlocation + 1
                break;

        }


        //int turns = (currentPosition-wantedFrontIndex+3) % 3;
        int turns = (wantedFrontIndex);

        telemetry.addLine("Auto Sort Telemetry");
        telemetry.addData("Green Location", greenLocation);
        telemetry.addData("Current Position", currentPosition);
        telemetry.addData("Wanted Front", wantedFrontIndex);
        telemetry.addData("Turns", turns);
        telemetry.update();

//        if (turns > 0) {
//            spindex.spinTurns(turns);
//            currentPosition = (currentPosition + turns) % 3;
//        }
        return turns;
    }
}



//        if (grnCount != 1) {
//            telemetry.addData("Error","Expect 1 Green",grnCount);
//            telemetry.update();
//            return;
//        }
//
//        telemetry.addLine("Auto Sort Telemetry:");
//        telemetry.addData("green Location", greenLocation);
//        telemetry.addData("targetMotif", target);
//
//
//        if (target == targetMotif.GPP) {
//            if (greenLocation == 1) {
//                spindex.spinTurns(2);
//            } else if (greenLocation == 2) {
//                spindex.spinTurns(1);
//            }
//        } else if (target == targetMotif.PGP) {
//            if (greenLocation == 0) {
//                spindex.spinTurns(1);
//            } else if (greenLocation == 2) {
//                spindex.spinTurns(2);
//            }
//        } else {
//            if (greenLocation == 0) {
//                spindex.spinTurns(2);
//            } else if (greenLocation == 1) {
//                spindex.spinTurns(1);
//            }
//        }
        //could also do
