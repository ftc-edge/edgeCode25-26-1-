package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.tensorflow.lite.Interpreter;
import org.tensorflow.lite.support.common.FileUtil;

import java.lang.reflect.InvocationTargetException;
import java.nio.MappedByteBuffer;
import java.util.Arrays;
import java.lang.reflect.Method;
@Autonomous
public class yaibaTensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry.addLine("Tensor Inspector (robust) starting...");
        telemetry.update();

        Interpreter interpreter = null;
        try {
            MappedByteBuffer model = FileUtil.loadMappedFile(hardwareMap.appContext, "BODY.tflite");
            Interpreter.Options options = new Interpreter.Options();
            options.setNumThreads(1);
            interpreter = new Interpreter(model, options);

            int inputCount = interpreter.getInputTensorCount();
            int outputCount = interpreter.getOutputTensorCount();

            telemetry.addData("Model", "Loaded OK");
            telemetry.addData("Input count", inputCount);
            telemetry.addData("Output count", outputCount);
            telemetry.update();

            // Show input tensors
            for (int i = 0; i < inputCount; i++) {
                String nameStr = inspectTensorPropertySafe(interpreter.getInputTensor(i), "name");
                String shapeStr = inspectTensorPropertySafe(interpreter.getInputTensor(i), "shape");
                String dtypeStr = inspectTensorPropertySafe(interpreter.getInputTensor(i), "dataType");
                telemetry.addLine(String.format("Input[%d]: name=%s shape=%s dtype=%s", i, nameStr, shapeStr, dtypeStr));
            }

            // Show output tensors
            for (int i = 0; i < outputCount; i++) {
                String nameStr = inspectTensorPropertySafe(interpreter.getOutputTensor(i), "name");
                String shapeStr = inspectTensorPropertySafe(interpreter.getOutputTensor(i), "shape");
                String dtypeStr = inspectTensorPropertySafe(interpreter.getOutputTensor(i), "dataType");
                telemetry.addLine(String.format("Output[%d]: name=%s shape=%s dtype=%s", i, nameStr, shapeStr, dtypeStr));
            }

            telemetry.addLine("Done. Press Start to finish.");
            telemetry.update();

        } catch (Exception e) {
            telemetry.addLine("Error loading/inspecting model:");
            telemetry.addData("Exception", e.getClass().getSimpleName() + ": " + e.getMessage());
            telemetry.update();
        }

        // Keep telemetry visible until user presses Start; then exit when OpMode is active.
        waitForStart();
        while (opModeIsActive()) {
            telemetry.update();
            sleep(100);
        }

        // Cleanup
        if (interpreter != null) {
            try {
                interpreter.close();
            } catch (Exception ignore) {}
        }
    }

    /**
     * Inspect a tensor object for a given property name. Supported propertyName values:
     *   - "name"     -> tries name(), getName()
     *   - "shape"    -> tries shape(), getShape(); returns formatted int[] or "ERR:..."
     *   - "dataType" -> tries dataType(), getDataType()
     *
     * If any reflection call throws, this returns a readable string "ERR:<ExceptionClass>: <message>".
     * If method not found, returns "N/A".
     */
    private String inspectTensorPropertySafe(Object tensorObj, String propertyName) {
        if (tensorObj == null) return "null";

        // Candidate method names by property
        String[] candidates;
        if ("name".equals(propertyName)) {
            candidates = new String[]{"name", "getName"};
        } else if ("shape".equals(propertyName)) {
            candidates = new String[]{"shape", "getShape"};
        } else if ("dataType".equals(propertyName)) {
            candidates = new String[]{"dataType", "getDataType"};
        } else {
            return "unknown_property";
        }

        for (String methodName : candidates) {
            try {
                Method m = tensorObj.getClass().getMethod(methodName);
                Object val = m.invoke(tensorObj);

                if (val == null) return "null";

                // Special handling for shape (int[])
                if ("shape".equals(propertyName)) {
                    if (val instanceof int[]) {
                        return Arrays.toString((int[]) val);
                    } else {
                        return val.toString(); // best-effort
                    }
                } else {
                    return val.toString();
                }
            } catch (NoSuchMethodException nsme) {
                // Try next candidate
                continue;
            } catch (IllegalAccessException iae) {
                return "ERR:IllegalAccessException: " + safeMsg(iae);
            } catch (InvocationTargetException ite) {
                Throwable cause = ite.getCause();
                String causeMsg = (cause != null) ? (cause.getClass().getSimpleName() + ": " + cause.getMessage()) :
                        "InvocationTargetException (no cause)";
                return "ERR:InvocationTargetException -> " + causeMsg;
            } catch (Exception e) {
                // Catch all -- return exception info
                return "ERR:" + e.getClass().getSimpleName() + ": " + safeMsg(e);
            }
        }

        // No candidate method found
        return "N/A";
    }

    private String safeMsg(Throwable t) {
        if (t == null) return "";
        String m = t.getMessage();
        return (m == null) ? "" : m;
    }
}

