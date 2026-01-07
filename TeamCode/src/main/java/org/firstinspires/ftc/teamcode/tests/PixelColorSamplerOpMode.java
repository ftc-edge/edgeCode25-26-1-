package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Mat;

@TeleOp(name = "Pixel Color Sampler (RGB->HSL)", group = "Vision")
public class PixelColorSamplerOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private PixelSamplerProcessor processor;

    @Override
    public void runOpMode() {
        processor = new PixelSamplerProcessor();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1"); // rename if needed

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .build();

        telemetry.addLine("Use D-pad to move sample point. RB = step 10. A = center. Press START to begin.");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                int step = gamepad1.right_bumper ? 10 : 1;

                if (gamepad1.dpad_left)  processor.nudge(-step, 0);
                if (gamepad1.dpad_right) processor.nudge(step, 0);
                if (gamepad1.dpad_up)    processor.nudge(0, -step);
                if (gamepad1.dpad_down)  processor.nudge(0, step);

                if (gamepad1.a) processor.center();

                // Read the latest sampled values (processor runs on a different thread)
                PixelSamplerProcessor.Sample s = processor.getLatestSample();

                telemetry.addData("Sample (x,y)", "%d, %d", s.x, s.y);
                telemetry.addData("RGB", "(%d, %d, %d)", s.r, s.g, s.b);
                telemetry.addData("HSL", "(%.1f°, %.1f%%, %.1f%%)", s.h, s.s, s.l);
                telemetry.addData("Frame", "%dx%d", processor.getFrameWidth(), processor.getFrameHeight());
                telemetry.update();

                sleep(20);
            }
        } finally {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    public static class PixelSamplerProcessor implements VisionProcessor {

        public static class Sample {
            public int x, y;
            public int r, g, b;
            public float h, s, l; // hue degrees, saturation %, lightness %
        }

        private final Object lock = new Object();
        private final Sample latest = new Sample();

        private int frameWidth = 0;
        private int frameHeight = 0;

        // default sample point (will be centered once we know width/height)
        private int sampleX = 0;
        private int sampleY = 0;

        private final Paint paint = new Paint();

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
            frameWidth = width;
            frameHeight = height;
            sampleX = width / 2;
            sampleY = height / 2;

            paint.setAntiAlias(true);
            paint.setStrokeWidth(4f);
            paint.setTextSize(36f);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            if (frame == null || frame.empty()) return null;
            if (frame.channels() < 3) return null;

            int x, y;
            synchronized (lock) {
                x = clamp(sampleX, 0, frame.cols() - 1);
                y = clamp(sampleY, 0, frame.rows() - 1);
            }

            // Mat.get(row=y, col=x). For RGB Mat, order is [R,G,B].
            double[] pix = frame.get(y, x);
            if (pix == null || pix.length < 3) return null;

            int r = clamp((int) Math.round(pix[0]), 0, 255);
            int g = clamp((int) Math.round(pix[1]), 0, 255);
            int b = clamp((int) Math.round(pix[2]), 0, 255);

            float[] hsl = rgbToHsl(r, g, b);

            synchronized (lock) {
                latest.x = x; latest.y = y;
                latest.r = r; latest.g = g; latest.b = b;
                latest.h = hsl[0]; latest.s = hsl[1]; latest.l = hsl[2];
            }

            return null; // no userContext needed
        }

        @Override
        public void onDrawFrame(
                Canvas canvas,
                int onscreenWidth,
                int onscreenHeight,
                float scaleBmpPxToCanvasPx,
                float scaleCanvasDensity,
                Object userContext
        ) {
            Sample s = getLatestSample();

            // Convert frame pixel coords -> canvas coords
            float cx = s.x * scaleBmpPxToCanvasPx;
            float cy = s.y * scaleBmpPxToCanvasPx;

            // Crosshair
            paint.setARGB(255, 0, 255, 0);
            canvas.drawLine(cx - 20, cy, cx + 20, cy, paint);
            canvas.drawLine(cx, cy - 20, cx, cy + 20, paint);

            // Text box
            paint.setARGB(255, 255, 255, 255);
            String line1 = String.format("RGB(%d,%d,%d)", s.r, s.g, s.b);
            String line2 = String.format("HSL(%.0f, %.0f%%, %.0f%%)", s.h, s.s, s.l);
            canvas.drawText(line1, 10, 40, paint);
            canvas.drawText(line2, 10, 80, paint);
        }

        public Sample getLatestSample() {
            synchronized (lock) {
                Sample copy = new Sample();
                copy.x = latest.x; copy.y = latest.y;
                copy.r = latest.r; copy.g = latest.g; copy.b = latest.b;
                copy.h = latest.h; copy.s = latest.s; copy.l = latest.l;
                return copy;
            }
        }

        public int getFrameWidth()  { return frameWidth; }
        public int getFrameHeight() { return frameHeight; }

        public void nudge(int dx, int dy) {
            synchronized (lock) {
                sampleX = clamp(sampleX + dx, 0, Math.max(0, frameWidth - 1));
                sampleY = clamp(sampleY + dy, 0, Math.max(0, frameHeight - 1));
            }
        }

        public void center() {
            synchronized (lock) {
                if (frameWidth > 0 && frameHeight > 0) {
                    sampleX = frameWidth / 2;
                    sampleY = frameHeight / 2;
                }
            }
        }

        private static int clamp(int v, int lo, int hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        /**
         * Converts 8-bit RGB to HSL.
         * Returns: [HueDegrees 0..360), [Saturation% 0..100], [Lightness% 0..100]
         */
        private static float[] rgbToHsl(int r, int g, int b) {
            float rf = r / 255f;
            float gf = g / 255f;
            float bf = b / 255f;

            float max = Math.max(rf, Math.max(gf, bf));
            float min = Math.min(rf, Math.min(gf, bf));
            float l = (max + min) / 2f;

            float h, s;
            if (max == min) {
                h = 0f;
                s = 0f;
            } else {
                float d = max - min;
                s = (l > 0.5f) ? (d / (2f - max - min)) : (d / (max + min));

                if (max == rf) {
                    h = (gf - bf) / d + (gf < bf ? 6f : 0f);
                } else if (max == gf) {
                    h = (bf - rf) / d + 2f;
                } else {
                    h = (rf - gf) / d + 4f;
                }
                h *= 60f;
                if (h >= 360f) h -= 360f;
            }

            return new float[] { h, s * 100f, l * 100f };
        }
    }
}
