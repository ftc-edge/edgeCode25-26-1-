package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * SpindexPID — Positional PID controller for the 2026 FTC DECODE Spindex
 *
 * Motor Specs:
 *   - 312 RPM goBILDA 5203 Series (or equivalent)
 *   - 537.7 Counts Per Revolution (CPR)
 *
 * Target per step:
 *   - 537.7 / 3 = 179.23 ticks  (1/3 rotation)
 *
 * PID Tuning Rationale:
 *   Kp = 0.0045  → At max error (179 ticks), output ≈ 0.81. Fast without slamming.
 *   Ki = 0.00004 → Tiny integral to eliminate steady-state error from friction.
 *                  Windup is capped at ±100 ticks to prevent runaway.
 *   Kd = 0.00028 → Dampens overshoot. Scaled to motor's ~2796 ticks/sec max velocity.
 *
 * Tolerance = 8 ticks (~5.4° arc) — tight enough for reliable indexing, forgiving
 *             enough not to hunt/oscillate at rest.
 *
 * Usage:
 *   SpindexPID spindex = new SpindexPID(hardwareMap.get(DcMotor.class, "spindexMotor"));
 *   spindex.advance();              // rotate 1/3 forward
 *   spindex.reverse();              // rotate 1/3 backward
 *   spindex.update();               // call every loop iteration
 *   boolean done = spindex.isAtTarget();
 */
@Config
public class SpindexPID {

    // ──────────────────────────────────────────────────────────────────────────
    //  Hardware Constants
    // ──────────────────────────────────────────────────────────────────────────

    public static double powerFactor = 1;
    /** Encoder counts per full motor revolution. */
    public static double TICKS_PER_REV     = 537.7;

    /** Encoder counts for exactly 1/3 of a revolution. */
    public static double TICKS_PER_STEP    = 128;  // 179.23

    /** Motor's free-run speed in RPM. */
    public static double MOTOR_RPM         = 312.0;

    /** Max ticks/sec at 100% power (used to tune Kd). */
    public static double MAX_TICKS_PER_SEC = (MOTOR_RPM / 60.0) * TICKS_PER_REV; // ≈2796


    // ──────────────────────────────────────────────────────────────────────────
    //  PID Gains  —  START HERE WHEN TUNING
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Proportional gain.
     * Effect:  Higher → faster, but may overshoot and oscillate.
     * Tuning:  Raise until you see overshoot, then back off ~20%.
     */
    public static double Kp = 0.0075;

    /**
     * Integral gain.
     * Effect:  Corrects residual steady-state error caused by friction.
     * Tuning:  Keep very small. Raise only if spindex consistently stops 5–15 ticks short.
     */
    public double Ki = 0.00004;

    /**
     * Derivative gain.
     * Effect:  Brakes the motor as it approaches the target to reduce overshoot.
     * Tuning:  Raise if you see oscillation after Kp is set. Lower if response feels sluggish.
     */
    public double Kd = 0.00028;

    /**
     * Acceptable position error in encoder ticks.
     * ±5 ticks ≈ ±3.4° of arc at the motor shaft.
     */
    public static int    POSITION_TOLERANCE  = 5;

    /**
     * Integral anti-windup cap. The accumulated integral is clamped to this
     * range so a long stall can't push the motor to full power unexpectedly.
     */
    public static double INTEGRAL_WINDUP_CAP = 100.0;

    /**
     * Minimum output power applied to the motor.
     * Helps the motor overcome static friction at very small errors.
     */
    public static double MIN_POWER           = 0.05;

    /**
     * Maximum output power applied to the motor.
     * Protects the mechanism from impact at full speed.
     */
    public static double MAX_POWER           = 1.0;

    public static double beforeShootAdjust = 0.5;
    public static double adjustDelay       = 0.450;
    public static double adjustDelay2      = 0.100;

    // ──────────────────────────────────────────────────────────────────────────
    //  Slow-Adjust Settings  (used during pre-shoot nudge for accuracy)
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Power factor applied during the pre-shoot adjustment nudge (shots 0–1).
     * Lower than the normal powerFactor so the motor settles precisely before
     * the shooter fires. Tune this until the nudge is smooth but not sluggish.
     * Range: 0.0 – 1.0.
     */
    public static double ADJUST_POWER_FACTOR = 0.5;

    // ──────────────────────────────────────────────────────────────────────────
    //  Jam Detection Settings
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Seconds without meaningful encoder movement before a jam is declared.
     * Start conservatively high (e.g. 0.6 s) and lower if jams go undetected.
     */
    public static double JAM_TIMEOUT_SECONDS  = 0.6;

    /**
     * Minimum encoder ticks of movement required within JAM_TIMEOUT_SECONDS
     * to prove the motor is still making progress. Below this threshold the
     * spindex is considered jammed.
     */
    public static int    JAM_MOVEMENT_TICKS   = 5;

    /**
     * How many back-and-retry cycles to attempt before giving up and holding
     * position. Prevents an infinite jam loop if the mechanism is truly stuck.
     */
    public static int    JAM_MAX_RETRIES      = 3;


    // ──────────────────────────────────────────────────────────────────────────
    //  Internal State
    // ──────────────────────────────────────────────────────────────────────────

    private final DcMotor    motor;
    private final ElapsedTime timer = new ElapsedTime();

    private int     targetPosition = 0;   // absolute encoder target in ticks
    private int     currentStep    = 0;   // which 1/3-rotation slot we're on (0, 1, 2 …)

    private double  lastError      = 0;
    private double  integralSum    = 0;
    private boolean atTarget       = true;

    private static ElapsedTime shootTimer = new ElapsedTime();
    private static ElapsedTime busyTimer  = new ElapsedTime();
    public  int     shot      = 0;
    public  boolean shooting  = false;

    // ── Jam detection state ───────────────────────────────────────────────────
    private final ElapsedTime jamTimer           = new ElapsedTime();
    private int               jamCheckPosition   = 0;   // encoder snapshot for movement check
    private int               preAttemptPosition = 0;   // position recorded when a new move starts
    private int               jamRetryCount      = 0;
    private boolean           isJammed           = false;


    // ──────────────────────────────────────────────────────────────────────────
    //  Constructor
    // ──────────────────────────────────────────────────────────────────────────

    public SpindexPID(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "spindex");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
        jamTimer.reset();
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  Public Control Methods
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Command the spindex to advance forward by exactly 1/3 rotation.
     * Safe to call even while a previous move is in progress — the new
     * target is stacked on top of the current one.
     */
    public void advance() {
        currentStep++;
        setTargetStep(currentStep);
    }

    /**
     * Command the spindex to reverse by exactly 1/3 rotation.
     */
    public void reverse() {
        currentStep--;
        setTargetStep(currentStep);
    }

    /**
     * Jump directly to a numbered slot (0-indexed).
     * Slot 0 = home/reset position.
     */
    public void goToSlot(int slot) {
        currentStep = slot;
        setTargetStep(currentStep);
    }

    /**
     * Reset the encoder and zero all state. Call when spindex is physically
     * at its home position.
     */
    public void resetHome() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        currentStep        = 0;
        targetPosition     = 0;
        lastError          = 0;
        integralSum        = 0;
        atTarget           = true;
        isJammed           = false;
        jamRetryCount      = 0;
        motor.setPower(0);
        timer.reset();
        jamTimer.reset();
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  PID Update  —  Call Every Loop Iteration
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Runs one PID iteration and performs jam detection. MUST be called once
     * per OpMode loop.
     *
     * Jam detection logic:
     *   Every JAM_TIMEOUT_SECONDS, the encoder position is compared against the
     *   snapshot taken at the start of that window. If the motor has moved fewer
     *   than JAM_MOVEMENT_TICKS it is declared jammed. The controller then
     *   commands a return to the position held before the stalled move and
     *   immediately re-issues the original target, up to JAM_MAX_RETRIES times.
     *   After exhausting retries the motor holds its current position and
     *   isJammed is set true.
     *
     * @return The motor power output applied this cycle (for telemetry).
     */
    public double update() {
        double dt = timer.seconds();
        timer.reset();

        // Guard against absurdly large dt (e.g., first loop after a long pause)
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        int    currentPosition = motor.getCurrentPosition();
        double error           = targetPosition - currentPosition;

        // ── Jam detection ─────────────────────────────────────────────────────
        if (!atTarget) {
            if (jamTimer.seconds() >= JAM_TIMEOUT_SECONDS) {
                int movement = Math.abs(currentPosition - jamCheckPosition);
                if (movement < JAM_MOVEMENT_TICKS) {
                    // Motor has not made meaningful progress — it's jammed.
                    if (jamRetryCount < JAM_MAX_RETRIES) {
                        jamRetryCount++;
                        int pendingTarget = targetPosition;          // remember where we were going
                        // Step 1 — back up to the pre-attempt position to clear the jam.
                        targetPosition = preAttemptPosition;
                        integralSum    = 0;
                        lastError      = 0;
                        // Step 2 — re-queue the original target so the next call to
                        //          setTargetStep (or the resume below) re-issues it.
                        // We do a two-phase approach: first move back, then on the
                        // next jam-window expiry (or once atTarget) re-issue the move.
                        // For simplicity we store the pending target and let the
                        // next jam window issue it after we reach preAttemptPosition.
                        jamRetryPendingTarget = pendingTarget;
                        jamRetryPending       = true;
                        jamTimer.reset();
                        jamCheckPosition = currentPosition;
                    } else {
                        // Exhausted retries — hold here and flag jammed.
                        isJammed           = true;
                        targetPosition     = currentPosition;
                        integralSum        = 0;
                        motor.setPower(0);
                        return 0;
                    }
                } else {
                    // Making progress — update snapshot and keep going.
                    jamCheckPosition = currentPosition;
                    jamTimer.reset();
                }
            }

            // Once we've returned to preAttemptPosition after a jam, re-issue
            // the original target so the retry move begins.
            if (jamRetryPending && Math.abs(currentPosition - preAttemptPosition) <= POSITION_TOLERANCE) {
                targetPosition    = jamRetryPendingTarget;
                jamRetryPending   = false;
                integralSum       = 0;
                lastError         = 0;
                preAttemptPosition = currentPosition;
                jamTimer.reset();
                jamCheckPosition   = currentPosition;
                error              = targetPosition - currentPosition;
            }
        }

        // ── Proportional ──────────────────────────────────────────────────────
        double pOut = Kp * error;

        // ── Integral (with anti-windup) ───────────────────────────────────────
        integralSum += error * dt;
        integralSum  = clamp(integralSum, -INTEGRAL_WINDUP_CAP, INTEGRAL_WINDUP_CAP);
        double iOut  = Ki * integralSum;

        // ── Derivative ────────────────────────────────────────────────────────
        double derivative = (error - lastError) / dt;
        double dOut        = Kd * derivative;
        lastError          = error;

        // ── Combine & clamp output ────────────────────────────────────────────
        double rawOutput = pOut + iOut + dOut;
        double output    = clamp(rawOutput, -MAX_POWER, MAX_POWER);

        // ── Dead-zone / at-target check ───────────────────────────────────────
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            atTarget      = true;
            integralSum   = 0;       // reset integral when settled
            jamRetryCount = 0;       // clear retry counter on successful arrival
            isJammed      = false;
            motor.setPower(0);
            return 0;
        }

        atTarget = false;

        // Apply minimum power so motor overcomes static friction
        if (Math.abs(output) < MIN_POWER) {
            output = Math.copySign(MIN_POWER, output);
        }

        motor.setPower(output * powerFactor);
        return output;
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  Status Queries
    // ──────────────────────────────────────────────────────────────────────────

    /** @return true when the motor is within POSITION_TOLERANCE of the target. */
    public boolean isAtTarget() { return atTarget; }

    /** @return Current encoder position in ticks. */
    public int getCurrentPosition() { return motor.getCurrentPosition(); }

    /** @return Current target position in ticks. */
    public int getTargetPosition() { return targetPosition; }

    /** @return Signed error (target − current) in ticks. */
    public double getError() { return targetPosition - motor.getCurrentPosition(); }

    /** @return The current 1/3-rotation step index. */
    public int getCurrentStep() { return currentStep; }

    /**
     * @return true if the motor has jammed and exhausted all retries.
     *         Call resetHome() or goToSlot() to clear this flag.
     */
    public boolean isJammed() { return isJammed; }

    /** @return How many jam-recovery attempts have been made on the current move. */
    public int getJamRetryCount() { return jamRetryCount; }


    // ──────────────────────────────────────────────────────────────────────────
    //  Shoot Consecutive
    // ──────────────────────────────────────────────────────────────────────────

    public void startShootConsecutive() {
        shooting = true;
        shot     = 0;
        shootTimer.reset();
    }

    /**
     * State-machine for the full shoot sequence. Call every loop after
     * startShootConsecutive().
     *
     * Sequence:
     *   shot 0  → nudge forward by beforeShootAdjust steps at ADJUST_POWER_FACTOR
     *             (slow, accurate seating against the shooter gap)
     *   shot 1  → wait adjustDelay, nudge back by beforeShootAdjust at ADJUST_POWER_FACTOR
     *             (slow return to the precise fire position)
     *   shot 2  → restore normal powerFactor, advance 3 steps to begin firing
     *   shot 3+ → continue one step at a time while sensor sees PURPLE or GREEN;
     *             stop after 3 balls or if sensor sees nothing expected
     *
     * @param spindexColor  Live color sensor reading from the spindex cup.
     */
    public void shootConsecutive(Color spindexColor) {
        if (!shooting) return;

        // No pre-shoot adjustment here — call shootConsecutiveAdjust() manually
        // via the dedicated button before triggering this sequence.

        // ── Shot 0: advance 3 steps to bring the first ball to the shooter ────
        if (shot == 0 && isAtTarget()) {
            setTargetStep(-4);
            shootTimer.reset();
            shot++;
            return;
        }

        // ── Shot 1+: advance one step per ball while sensor sees a ball ───────
        if (shot >= 1 && isAtTarget() && shootTimer.seconds() >= adjustDelay2) {
            if (spindexColor.getColor().equals("PURPLE") || spindexColor.getColor().equals("GREEN")) {
                // Another ball is present — advance one more step
                setTargetStep(-1);
                shootTimer.reset();
                shot++;
            } else if (shot > 4) {
                // All 3 balls fired
                finishShooting();
            } else {
                // Sensor sees nothing expected — done
                finishShooting();
            }
        }
    }

    /**
     * Runs only the slow pre-shoot adjustment (shots 0 and 1), without
     * continuing into the firing sequence. Useful when you want to seat
     * the balls precisely before triggering the shooter separately.
     *
     * Restores the original powerFactor once both nudges are complete.
     *
     * @return true once both nudges have settled (adjustment complete).
     */
    public boolean shootConsecutiveAdjust() {
        if (shot == 0) {
            powerFactor = ADJUST_POWER_FACTOR;
            setTargetStep(beforeShootAdjust);
            shootTimer.reset();
            shot++;
            return false;
        }
        if (shot == 1 && isAtTarget() && shootTimer.seconds() >= adjustDelay) {
            setTargetStep(-beforeShootAdjust);
            shootTimer.reset();
            shot++;
            return false;
        }
        if (shot >= 2 && isAtTarget()) {
            powerFactor = 1.0;    // restore full speed now that adjustment is done
            return true;
        }
        return false;
    }

    // ──────────────────────────────────────────────────────────────────────────
    //  Jam Detection — Internal State (declared here so update() can see them)
    // ──────────────────────────────────────────────────────────────────────────

    private int     jamRetryPendingTarget = 0;
    private boolean jamRetryPending       = false;


    // ──────────────────────────────────────────────────────────────────────────
    //  Helpers
    // ──────────────────────────────────────────────────────────────────────────

    public void setTargetStep(double step) {
        preAttemptPosition = motor.getCurrentPosition();   // snapshot for jam detection
        targetPosition    += (int) Math.round(step * TICKS_PER_STEP);
        atTarget           = false;
        integralSum        = 0;
        isJammed           = false;
        jamRetryCount      = 0;
        jamRetryPending    = false;
        jamCheckPosition   = preAttemptPosition;
        jamTimer.reset();
        timer.reset();
    }

    public void spin(double ticks) {
        preAttemptPosition = motor.getCurrentPosition();
        targetPosition    += (int) ticks;
        atTarget           = false;
        integralSum        = 0;
        isJammed           = false;
        jamRetryCount      = 0;
        jamRetryPending    = false;
        jamCheckPosition   = preAttemptPosition;
        jamTimer.reset();
        timer.reset();
    }

    public void stop() {
        motor.setPower(0);
    }

    private void finishShooting() {
        powerFactor = 1.0;
        shooting    = false;
        shot        = 0;
        shootTimer.reset();
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}