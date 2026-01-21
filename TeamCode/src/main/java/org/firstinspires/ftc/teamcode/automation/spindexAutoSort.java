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

    public void sortNShoot(int[] layout, targetMotif target) {
        int grnCount = 0;
        int purCount = 0;
        int greenLocation = -1;
        for (int i = 0; i <= 2; i++) {
            if (layout[i] == 1) {
                grnCount++;
                greenLocation = i;
            }
            if (layout[i] == -1) {
                purCount++;
            }
        }
        if (grnCount != 1) {
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

    }
}
