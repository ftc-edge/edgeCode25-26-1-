package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.components.Spindex.beforeShootAdjust;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
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
     * ±8 ticks ≈ ±5.4° of arc at the motor shaft.
     */
    public static int   POSITION_TOLERANCE  = 1;

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

    public static int beforeShootAdjust = 60;
    public static int adjustDelayMs = 700;
    public static int adjustDelay2Ms = 450;


    // ──────────────────────────────────────────────────────────────────────────
    //  Internal State
    // ──────────────────────────────────────────────────────────────────────────

    private final DcMotor   motor;
    private final ElapsedTime timer = new ElapsedTime();

    private int    targetPosition = 0;   // absolute encoder target in ticks
    private int    currentStep    = 0;   // which 1/3-rotation slot we're on (0, 1, 2 …)

    private double lastError      = 0;
    private double integralSum    = 0;
    private boolean atTarget      = true;

    private static ElapsedTime shootTimer = new ElapsedTime();
    private static ElapsedTime busyTimer = new ElapsedTime();
    public int shot = 0;
    public boolean shooting = false;



    // ──────────────────────────────────────────────────────────────────────────
    //  Constructor
    // ──────────────────────────────────────────────────────────────────────────


    public SpindexPID(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotor.class, "spindex");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        timer.reset();
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
        currentStep    = 0;
        targetPosition = 0;
        lastError      = 0;
        integralSum    = 0;
        atTarget       = true;
        motor.setPower(0);
        timer.reset();
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  PID Update  —  Call Every Loop Iteration
    // ──────────────────────────────────────────────────────────────────────────

    /**
     * Runs one PID iteration. MUST be called once per OpMode loop.
     *
     * @return The motor power output applied this cycle (for telemetry).
     */
    public double update() {
        double dt = timer.seconds();
        timer.reset();

        // Guard against absurdly large dt (e.g., first loop after a long pause)
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        int currentPosition = motor.getCurrentPosition();
        double error = targetPosition - currentPosition;

        // ── Proportional ──────────────────────────────────────────────────
        double pOut = Kp * error;

        // ── Integral (with anti-windup) ───────────────────────────────────
        integralSum += error * dt;
        integralSum  = clamp(integralSum, -INTEGRAL_WINDUP_CAP, INTEGRAL_WINDUP_CAP);
        double iOut  = Ki * integralSum;

        // ── Derivative ────────────────────────────────────────────────────
        double derivative = (error - lastError) / dt;
        double dOut        = Kd * derivative;
        lastError          = error;

        // ── Combine & clamp output ─────────────────────────────────────────
        double rawOutput = pOut + iOut + dOut;
        double output    = clamp(rawOutput, -MAX_POWER, MAX_POWER);

        // ── Dead-zone / at-target check ───────────────────────────────────
        if (Math.abs(error) <= POSITION_TOLERANCE) {
            atTarget    = true;
            integralSum = 0;        // reset integral when settled
            motor.setPower(0);
            return 0;
        }

        atTarget = false;

        // Apply minimum power so motor overcomes static friction
        if (Math.abs(output) < MIN_POWER) {
            output = Math.copySign(MIN_POWER, output);
        }

        motor.setPower(output);
        return output;
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  Status Queries
    // ──────────────────────────────────────────────────────────────────────────

    /** @return true when the motor is within POSITION_TOLERANCE of the target. */
    public boolean isAtTarget() {
        return atTarget;
    }

    /** @return Current encoder position in ticks. */
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    /** @return Current target position in ticks. */
    public int getTargetPosition() {
        return targetPosition;
    }

    /** @return Signed error (target − current) in ticks. */
    public double getError() {
        return targetPosition - motor.getCurrentPosition();
    }

    /** @return The current 1/3-rotation step index. */
    public int getCurrentStep() {
        return currentStep;
    }


    // ──────────────────────────────────────────────────────────────────────────
    //  Private Helpers
    // ──────────────────────────────────────────────────────────────────────────

    public void startShootConsecutive(){
        shooting = true;
        shot = 0;
        shootTimer.reset();
    }

    public void shootConsecutive(Color spindexColor){
        if (!shooting){
            return;
        }

        // 0th stage, turn back a little
        // first stage, turn forward a little
        // 2-4 stage, shoot each ball
        if (shot == 0) {
            spin(beforeShootAdjust);
            shootTimer.reset(); shot++; return;
        }
        if (shot == 1 && shootTimer.milliseconds() >= adjustDelayMs) {
            spin(-beforeShootAdjust);
            shootTimer.reset(); shot++; return;
        }
        if(shot >= 2 && isAtTarget() && shootTimer.milliseconds() >= adjustDelay2Ms){
            if(shot == 2){
                setTargetStep(-3);
                shootTimer.reset();
                shot++;
            }
            else if(spindexColor.getColor() == "PURPLE" || spindexColor.getColor() == "GREEN"){
                setTargetStep(-1);
                shootTimer.reset();
                shot++;
            }else if(shot > 5){
                shootTimer.reset();
                shooting = false;
            }else{
                shootTimer.reset();
                shooting = false;
            }
            return;
        }
//            if(shot <= 4){ // 2, 3, 4, we dont check
//                spinUp();
//                shootTimer.reset();
//                shot++;
//            }
//            else if(spindexColor.getColor() == "PURPLE" || spindexColor.getColor() == "GREEN"){
//                spinUp();
//                shootTimer.reset();
//                shot++;
//            }else{
//                shot = -4;
//                shootTimer.reset();
//            }
//        }
//        if(shot < -1 && shootTimer.milliseconds() >= readColorDelayMs){ // adjust so that the camera reads all ball positions, shot = -4, -3, -2
//            shot++;
//            spinTurns(1);
//            shootTimer.reset();
//        }
//        if(shot == -1) {
//            shooting = false;
//            shot = 0;
//            shootTimer.reset();
//        }
    }
    public void setTargetStep(int step) {
        targetPosition += (int) Math.round(step * TICKS_PER_STEP);
        atTarget       = false;
        integralSum    = 0;   // reset integral on new command
        timer.reset();
    }

    public void spin(double ticks){
        targetPosition += ticks;
        atTarget = false;
        integralSum = 0;
        timer.reset();
    }

    public void stop(){
        motor.setPower(0);
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}