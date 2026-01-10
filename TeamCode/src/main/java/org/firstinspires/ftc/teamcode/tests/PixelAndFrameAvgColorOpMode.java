package org.firstinspires.ftc.teamcode.tests;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

@TeleOp(name = "Pixel + Frame Avg Color (HSL + Rolling)", group = "Vision")
public class PixelAndFrameAvgColorOpMode extends LinearOpMode {

    private VisionPortal visionPortal;
    private PixelAndFrameAvgProcessor processor;

    @Override
    public void runOpMode() {
        processor = new PixelAndFrameAvgProcessor(6); // rolling window = 6 frames

        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1"); // rename if needed

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .build();

        telemetry.addLine("Controls: D-pad moves sample point. RB = step 10. A = center. ESC/Stop to end.");
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

                PixelAndFrameAvgProcessor.Sample s = processor.getLatestSample();

                telemetry.addData("Sample (x,y)", "%d, %d", s.x, s.y);

                telemetry.addLine("Target pixel (current):");
                telemetry.addData("  RGB", "(%d, %d, %d)", s.r, s.g, s.b);
                telemetry.addData("  HSL", "(%.1f°, %.1f%%, %.1f%%)", s.h, s.s, s.l);

                telemetry.addLine("Target pixel (rolling 6):");
                telemetry.addData("  RGB", "(%d, %d, %d)", s.rRoll, s.gRoll, s.bRoll);
                telemetry.addData("  HSL", "(%.1f°, %.1f%%, %.1f%%)", s.hRoll, s.sRoll, s.lRoll);

                telemetry.addLine("Whole frame mean (current):");
                telemetry.addData("  RGB", "(%d, %d, %d)", s.frameMeanR, s.frameMeanG, s.frameMeanB);
                telemetry.addData("  HSL", "(%.1f°, %.1f%%, %.1f%%)", s.frameMeanH, s.frameMeanS, s.frameMeanL);

                telemetry.addLine("Whole frame mean (rolling 6):");
                telemetry.addData("  RGB", "(%d, %d, %d)", s.frameMeanRRoll, s.frameMeanGRoll, s.frameMeanBRoll);
                telemetry.addData("  HSL", "(%.1f°, %.1f%%, %.1f%%)", s.frameMeanHRoll, s.frameMeanSRoll, s.frameMeanLRoll);

                telemetry.addData("Frame WxH", "%dx%d", processor.getFrameWidth(), processor.getFrameHeight());
                telemetry.update();

                sleep(20);
            }
        } finally {
            if (visionPortal != null) visionPortal.close();
        }
    }

    public static class PixelAndFrameAvgProcessor implements VisionProcessor {

        public static class Sample {
            // target pixel (current)
            public int x, y;
            public int r, g, b;
            public float h, s, l;

            // target pixel (rolling)
            public int rRoll, gRoll, bRoll;
            public float hRoll, sRoll, lRoll;

            // whole frame mean (current)
            public int frameMeanR, frameMeanG, frameMeanB;
            public float frameMeanH, frameMeanS, frameMeanL;

            // whole frame mean (rolling)
            public int frameMeanRRoll, frameMeanGRoll, frameMeanBRoll;
            public float frameMeanHRoll, frameMeanSRoll, frameMeanLRoll;
        }

        private final Object lock = new Object();
        private final Sample latest = new Sample();

        private int frameWidth = 0;
        private int frameHeight = 0;

        private int sampleX = 0;
        private int sampleY = 0;

        private final Paint paint = new Paint();

        // rolling window
        private final int windowN;

        // target rolling buffers + sums
        private final int[] tgtRBuf, tgtGBuf, tgtBBuf;
        private long tgtRSum = 0, tgtGSum = 0, tgtBSum = 0;

        // frame-mean rolling buffers + sums (store as ints 0..255)
        private final int[] meanRBuf, meanGBuf, meanBBuf;
        private long meanRSum = 0, meanGSum = 0, meanBSum = 0;

        private int rollCount = 0;   // number of valid samples in buffers (<= windowN)
        private int rollIndex = 0;   // circular index

        public PixelAndFrameAvgProcessor(int rollingWindowFrames) {
            this.windowN = Math.max(1, rollingWindowFrames);
            tgtRBuf = new int[windowN];
            tgtGBuf = new int[windowN];
            tgtBBuf = new int[windowN];

            meanRBuf = new int[windowN];
            meanGBuf = new int[windowN];
            meanBBuf = new int[windowN];
        }

        @Override
        public void init(int width, int height, org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
            frameWidth = width;
            frameHeight = height;
            sampleX = width / 2;
            sampleY = height / 2;

            paint.setAntiAlias(true);
            paint.setStrokeWidth(4f);
            paint.setTextSize(36f);

            synchronized (lock) {
                resetRollingLocked();
            }
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

            // Read target pixel: frame.get(row=y, col=x)
            double[] pix = frame.get(y, x);
            if (pix == null || pix.length < 3) return null;

            // Assumption: Mat is RGB (common in FTC VisionProcessor pipelines).
            int r = clamp((int) Math.round(pix[0]), 0, 255);
            int g = clamp((int) Math.round(pix[1]), 0, 255);
            int b = clamp((int) Math.round(pix[2]), 0, 255);
            float[] hsl = rgbToHsl(r, g, b);

            // Whole-frame mean (average of all pixels)
            Scalar m = Core.mean(frame); // per-channel mean
            int meanR = clamp((int) Math.round(m.val[0]), 0, 255);
            int meanG = clamp((int) Math.round(m.val[1]), 0, 255);
            int meanB = clamp((int) Math.round(m.val[2]), 0, 255);
            float[] meanHsl = rgbToHsl(meanR, meanG, meanB);

            // Update rolling buffers (target + mean)
            int rRoll, gRoll, bRoll;
            int meanRRoll, meanGRoll, meanBRoll;

            synchronized (lock) {
                // if buffer full, subtract outgoing element
                if (rollCount == windowN) {
                    tgtRSum -= tgtRBuf[rollIndex];
                    tgtGSum -= tgtGBuf[rollIndex];
                    tgtBSum -= tgtBBuf[rollIndex];

                    meanRSum -= meanRBuf[rollIndex];
                    meanGSum -= meanGBuf[rollIndex];
                    meanBSum -= meanBBuf[rollIndex];
                } else {
                    rollCount++;
                }

                // write incoming
                tgtRBuf[rollIndex] = r;  tgtGBuf[rollIndex] = g;  tgtBBuf[rollIndex] = b;
                meanRBuf[rollIndex] = meanR; meanGBuf[rollIndex] = meanG; meanBBuf[rollIndex] = meanB;

                // add to sums
                tgtRSum += r;  tgtGSum += g;  tgtBSum += b;
                meanRSum += meanR; meanGSum += meanG; meanBSum += meanB;

                // advance circular index
                rollIndex = (rollIndex + 1) % windowN;

                // compute rolling averages (integer RGB)
                rRoll = (int) Math.round((double) tgtRSum / rollCount);
                gRoll = (int) Math.round((double) tgtGSum / rollCount);
                bRoll = (int) Math.round((double) tgtBSum / rollCount);

                meanRRoll = (int) Math.round((double) meanRSum / rollCount);
                meanGRoll = (int) Math.round((double) meanGSum / rollCount);
                meanBRoll = (int) Math.round((double) meanBSum / rollCount);

                float[] hslRoll = rgbToHsl(rRoll, gRoll, bRoll);
                float[] meanHslRoll = rgbToHsl(meanRRoll, meanGRoll, meanBRoll);

                // publish latest snapshot
                latest.x = x; latest.y = y;

                latest.r = r; latest.g = g; latest.b = b;
                latest.h = hsl[0]; latest.s = hsl[1]; latest.l = hsl[2];

                latest.rRoll = rRoll; latest.gRoll = gRoll; latest.bRoll = bRoll;
                latest.hRoll = hslRoll[0]; latest.sRoll = hslRoll[1]; latest.lRoll = hslRoll[2];

                latest.frameMeanR = meanR; latest.frameMeanG = meanG; latest.frameMeanB = meanB;
                latest.frameMeanH = meanHsl[0]; latest.frameMeanS = meanHsl[1]; latest.frameMeanL = meanHsl[2];

                latest.frameMeanRRoll = meanRRoll; latest.frameMeanGRoll = meanGRoll; latest.frameMeanBRoll = meanBRoll;
                latest.frameMeanHRoll = meanHslRoll[0]; latest.frameMeanSRoll = meanHslRoll[1]; latest.frameMeanLRoll = meanHslRoll[2];
            }

            return null;
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

            float cx = s.x * scaleBmpPxToCanvasPx;
            float cy = s.y * scaleBmpPxToCanvasPx;

            // crosshair
            paint.setARGB(255, 0, 255, 0);
            canvas.drawLine(cx - 20, cy, cx + 20, cy, paint);
            canvas.drawLine(cx, cy - 20, cx, cy + 20, paint);

            // overlay text (kept short)
            paint.setARGB(255, 255, 255, 255);
            canvas.drawText(String.format("Target RGB(%d,%d,%d)", s.r, s.g, s.b), 10, 40, paint);
            canvas.drawText(String.format("Target6 RGB(%d,%d,%d)", s.rRoll, s.gRoll, s.bRoll), 10, 80, paint);
            canvas.drawText(String.format("Mean RGB(%d,%d,%d)", s.frameMeanR, s.frameMeanG, s.frameMeanB), 10, 120, paint);
            canvas.drawText(String.format("Mean6 RGB(%d,%d,%d)", s.frameMeanRRoll, s.frameMeanGRoll, s.frameMeanBRoll), 10, 160, paint);
        }

        public Sample getLatestSample() {
            synchronized (lock) {
                Sample c = new Sample();
                c.x = latest.x; c.y = latest.y;

                c.r = latest.r; c.g = latest.g; c.b = latest.b;
                c.h = latest.h; c.s = latest.s; c.l = latest.l;

                c.rRoll = latest.rRoll; c.gRoll = latest.gRoll; c.bRoll = latest.bRoll;
                c.hRoll = latest.hRoll; c.sRoll = latest.sRoll; c.lRoll = latest.lRoll;

                c.frameMeanR = latest.frameMeanR; c.frameMeanG = latest.frameMeanG; c.frameMeanB = latest.frameMeanB;
                c.frameMeanH = latest.frameMeanH; c.frameMeanS = latest.frameMeanS; c.frameMeanL = latest.frameMeanL;

                c.frameMeanRRoll = latest.frameMeanRRoll; c.frameMeanGRoll = latest.frameMeanGRoll; c.frameMeanBRoll = latest.frameMeanBRoll;
                c.frameMeanHRoll = latest.frameMeanHRoll; c.frameMeanSRoll = latest.frameMeanSRoll; c.frameMeanLRoll = latest.frameMeanLRoll;

                return c;
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

        private void resetRollingLocked() {
            tgtRSum = tgtGSum = tgtBSum = 0;
            meanRSum = meanGSum = meanBSum = 0;
            rollCount = 0;
            rollIndex = 0;
            for (int i = 0; i < windowN; i++) {
                tgtRBuf[i] = tgtGBuf[i] = tgtBBuf[i] = 0;
                meanRBuf[i] = meanGBuf[i] = meanBBuf[i] = 0;
            }
        }

        private static int clamp(int v, int lo, int hi) {
            return Math.max(lo, Math.min(hi, v));
        }

        /**
         * RGB (0..255) -> HSL:
         * returns [HueDegrees 0..360), [Saturation% 0..100], [Lightness% 0..100]
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
