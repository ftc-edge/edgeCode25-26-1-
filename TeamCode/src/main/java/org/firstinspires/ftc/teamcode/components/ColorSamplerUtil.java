package org.firstinspires.ftc.teamcode.components;

import android.graphics.Canvas;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

/**
 * Utility for sampling a target pixel color and whole-frame mean color from a camera feed,
 * with rolling averages over the last N frames. Designed to be used from another OpMode.
 *
 * Assumption: incoming Mat channel order is RGB (common in FTC VisionPortal pipelines).
 * If you find colors appear swapped, see note near readPixelRGB().
 */
public class ColorSamplerUtil {

    public static class Sample {
        // Target pixel (current)
        public int x, y;
        public int r, g, b;
        public float h, s, l;

        // Target pixel (rolling)
        public int rRoll, gRoll, bRoll;
        public float hRoll, sRoll, lRoll;

        // Whole frame mean (current)
        public int frameMeanR, frameMeanG, frameMeanB;
        public float frameMeanH, frameMeanS, frameMeanL;

        // Whole frame mean (rolling)
        public int frameMeanRRoll, frameMeanGRoll, frameMeanBRoll;
        public float frameMeanHRoll, frameMeanSRoll, frameMeanLRoll;
    }

    private final VisionPortal visionPortal;
    private final Processor processor;

    /**
     * Create and start the sampler.
     *
     * @param hardwareMap FTC HardwareMap
     * @param webcamName  name in RC configuration (e.g., "Webcam 1")
     * @param rollingN    rolling window size in frames (e.g., 6)
     */
    public ColorSamplerUtil(HardwareMap hardwareMap, String webcamName, int rollingN) {
        this.processor = new Processor(Math.max(1, rollingN));

        WebcamName webcam = hardwareMap.get(WebcamName.class, webcamName);

        this.visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(processor)
                .build();
    }

    /** Call when done (e.g., in finally or after loop). */
    public void close() {
        if (visionPortal != null) visionPortal.close();
    }

    /** Move sample point by dx,dy pixels (clamped to frame). */
    public void nudge(int dx, int dy) {
        processor.nudge(dx, dy);
    }

    /** Center sample point. */
    public void center() {
        processor.center();
    }

    /** Get latest computed values (thread-safe copy). */
    public Sample getSample() {
        return processor.getLatestSample();
    }

    /** Optional: enable/disable preview overlay (crosshair + text). */
    public void setOverlayEnabled(boolean enabled) {
        processor.setOverlayEnabled(enabled);
    }

    /** Get last known frame dimensions (0 if not initialized yet). */
    public int getFrameWidth()  { return processor.getFrameWidth(); }
    public int getFrameHeight() { return processor.getFrameHeight(); }

    // ---------------- Internal VisionProcessor ----------------

    private static class Processor implements VisionProcessor {

        private final Object lock = new Object();
        private final Sample latest = new Sample();

        private int frameWidth = 0;
        private int frameHeight = 0;

        private int sampleX = 0;
        private int sampleY = 0;

        private boolean overlayEnabled = true;

        private final Paint paint = new Paint();

        // rolling window
        private final int windowN;

        // target rolling buffers + sums
        private final int[] tgtRBuf, tgtGBuf, tgtBBuf;
        private long tgtRSum = 0, tgtGSum = 0, tgtBSum = 0;

        // frame mean rolling buffers + sums
        private final int[] meanRBuf, meanGBuf, meanBBuf;
        private long meanRSum = 0, meanGSum = 0, meanBSum = 0;

        private int rollCount = 0;
        private int rollIndex = 0;

        Processor(int windowN) {
            this.windowN = windowN;
            tgtRBuf = new int[windowN];
            tgtGBuf = new int[windowN];
            tgtBBuf = new int[windowN];

            meanRBuf = new int[windowN];
            meanGBuf = new int[windowN];
            meanBBuf = new int[windowN];
        }

        @Override
        public void init(int width, int height,
                         org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration calibration) {
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
            if (frame == null || frame.empty() || frame.channels() < 3) return null;

            final int cols = frame.cols();
            final int rows = frame.rows();

            int x, y;
            synchronized (lock) {
                x = clamp(sampleX, 0, cols - 1);
                y = clamp(sampleY, 0, rows - 1);
            }

            // --- target pixel ---
            int[] rgb = readPixelRGB(frame, x, y);
            int r = rgb[0], g = rgb[1], b = rgb[2];
            float[] hsl = rgbToHsl(r, g, b);

            // --- whole-frame mean ---
            Scalar m = Core.mean(frame); // per-channel mean
            int meanR = clamp((int) Math.round(m.val[0]), 0, 255);
            int meanG = clamp((int) Math.round(m.val[1]), 0, 255);
            int meanB = clamp((int) Math.round(m.val[2]), 0, 255);
            float[] meanHsl = rgbToHsl(meanR, meanG, meanB);

            synchronized (lock) {
                // Maintain rolling buffers (target + mean)
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

                tgtRBuf[rollIndex] = r;  tgtGBuf[rollIndex] = g;  tgtBBuf[rollIndex] = b;
                meanRBuf[rollIndex] = meanR; meanGBuf[rollIndex] = meanG; meanBBuf[rollIndex] = meanB;

                tgtRSum += r;  tgtGSum += g;  tgtBSum += b;
                meanRSum += meanR; meanGSum += meanG; meanBSum += meanB;

                rollIndex = (rollIndex + 1) % windowN;

                int rRoll = (int) Math.round((double) tgtRSum / rollCount);
                int gRoll = (int) Math.round((double) tgtGSum / rollCount);
                int bRoll = (int) Math.round((double) tgtBSum / rollCount);

                int meanRRoll = (int) Math.round((double) meanRSum / rollCount);
                int meanGRoll = (int) Math.round((double) meanGSum / rollCount);
                int meanBRoll = (int) Math.round((double) meanBSum / rollCount);

                float[] hslRoll = rgbToHsl(rRoll, gRoll, bRoll);
                float[] meanHslRoll = rgbToHsl(meanRRoll, meanGRoll, meanBRoll);

                // publish
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
        public void onDrawFrame(Canvas canvas,
                                int onscreenWidth,
                                int onscreenHeight,
                                float scaleBmpPxToCanvasPx,
                                float scaleCanvasDensity,
                                Object userContext) {

            if (!overlayEnabled) return;

            Sample s = getLatestSample();

            float cx = s.x * scaleBmpPxToCanvasPx;
            float cy = s.y * scaleBmpPxToCanvasPx;

            // Crosshair
            paint.setARGB(255, 0, 255, 0);
            canvas.drawLine(cx - 20, cy, cx + 20, cy, paint);
            canvas.drawLine(cx, cy - 20, cx, cy + 20, paint);

            // Minimal overlay text
            paint.setARGB(255, 255, 255, 255);
            canvas.drawText(String.format("T RGB(%d,%d,%d)  T6 RGB(%d,%d,%d)",
                    s.r, s.g, s.b, s.rRoll, s.gRoll, s.bRoll), 10, 40, paint);
            canvas.drawText(String.format("M RGB(%d,%d,%d)  M6 RGB(%d,%d,%d)",
                    s.frameMeanR, s.frameMeanG, s.frameMeanB,
                    s.frameMeanRRoll, s.frameMeanGRoll, s.frameMeanBRoll), 10, 80, paint);
        }

        Sample getLatestSample() {
            synchronized (lock) {
                Sample c = new Sample();
                // copy all fields
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

        int getFrameWidth()  { return frameWidth; }
        int getFrameHeight() { return frameHeight; }

        void setOverlayEnabled(boolean enabled) {
            synchronized (lock) { overlayEnabled = enabled; }
        }

        void nudge(int dx, int dy) {
            synchronized (lock) {
                sampleX = clamp(sampleX + dx, 0, Math.max(0, frameWidth - 1));
                sampleY = clamp(sampleY + dy, 0, Math.max(0, frameHeight - 1));
            }
        }

        void center() {
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
         * Read one pixel as RGB ints 0..255.
         * Assumes frame.get(y,x) returns [R,G,B]. If your preview looks swapped, try swapping:
         *   r = pix[2]; g = pix[1]; b = pix[0];
         */
        private static int[] readPixelRGB(Mat frame, int x, int y) {
            double[] pix = frame.get(y, x);
            if (pix == null || pix.length < 3) return new int[]{0, 0, 0};

            int r = clamp((int) Math.round(pix[0]), 0, 255);
            int g = clamp((int) Math.round(pix[1]), 0, 255);
            int b = clamp((int) Math.round(pix[2]), 0, 255);
            return new int[]{r, g, b};
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

            return new float[]{h, s * 100f, l * 100f};
        }
    }

}
