package org.firstinspires.ftc.teamcode.tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.yaiba.BODY;
import org.firstinspires.ftc.teamcode.yaiba.ModelInputMapper;


@Autonomous
public class modelInputTest extends OpMode {
    private BODY yaiba;
    @Override
    public void init() {
        yaiba = BODY.create(hardwareMap.appContext);
        if (yaiba == null) {
            telemetry.addData("MODEL", "Failed to load BODY.tflite");
            telemetry.addLine("Check: app/src/main/assets/BODY.tflite");
            telemetry.update();
            // Keep going but make sure any inference calls are guarded (yaiba != null).
        } else {
            telemetry.addData("MODEL", "Loaded successfully");
            telemetry.update();
        }
//        ModelInputMapper.MapperResult res = ModelInputMapper.createInputsFromFlat(interpreter, flatObs);
    }

    @Override
    public void loop() {
    }
}
