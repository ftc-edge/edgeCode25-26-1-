package org.firstinspires.ftc.teamcode.components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class TurretAutoAimODO {
    String mode = "teleop";
    public TurretRTP rtp;
    public GoBildaPinpointDriver odo;

    public static String currentAlliance = "BLUE";
    public static double SCALAR_FACTOR = 0.0;
    public static double HEADING_SCALAR;

    // CRITICAL: Set these to your actual goal position on the field!
    public static double GOAL_X_TELEOP = 1.0;  // CHANGE THIS
    public static double GOAL_Y_TELEOP = -1.0;  // CHANGE THIS

    public static double GOAL_X_AUTO = -1.0;
    public static double GOAL_Y_AUTO = -1.0;

//    public static double MOTIF_X_AUTO = 1;
//    public static double MOTIF_Y_AUTO = 0;
//
//    public static double MOTIF_X_TELEOP = 0;
//    public static double MOTIF_Y_TELEOP = -1;

    String target = "goal";

    double GOAL_X;
    double GOAL_Y;

    // Turret offset on robot (in USER degrees) - if turret 0° is not robot front
    public static double TURRET_OFFSET_DEGREES = 0.0;

    public TurretAutoAimODO(HardwareMap hardwareMap, double startX, double startY, String mode){
        rtp = new TurretRTP(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.mode = mode;

        if(currentAlliance.equals("RED")){
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setOffsets(12, -17.5, DistanceUnit.CM);
            odo.setPosition(new Pose2D(DistanceUnit.CM, startX, startY / AutoBlueConstants.MODEL_POS_SCALE, AngleUnit.RADIANS, 1.578));
            HEADING_SCALAR = -1;
        }else{
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
            odo.setOffsets(12, -17.5, DistanceUnit.CM);
            odo.setPosition(new Pose2D(DistanceUnit.CM, startX, startY / AutoBlueConstants.MODEL_POS_SCALE, AngleUnit.DEGREES, -1.578));
            HEADING_SCALAR = 1;
        }

        setTargetToGoal();
    }

    public void setTargetToGoal(){
        target = "goal";
        if (mode == "teleop"){
            if(Util.getColor().equals("blue")){
                GOAL_X = GOAL_X_TELEOP;
                GOAL_Y = GOAL_Y_TELEOP;
            } else {
                GOAL_X = -GOAL_X_TELEOP;
                GOAL_Y = GOAL_Y_TELEOP;
            }
        }else{
            if(Util.getColor().equals("blue")){
                GOAL_X = GOAL_X_AUTO;
                GOAL_Y = GOAL_Y_AUTO;
            } else {
                GOAL_X = GOAL_X_AUTO;
                GOAL_Y = -GOAL_Y_AUTO;
            }

        }
    }

    public void setTargetToMotif(){
        target = "motif";
//        if (mode == "teleop"){
//            GOAL_X = MOTIF_X_TELEOP;
//            GOAL_Y = MOTIF_Y_TELEOP;
//        }else{
//            GOAL_X = MOTIF_X_AUTO;
//            GOAL_Y = MOTIF_Y_AUTO;
//        }
    }

    public void runToAim(Telemetry telemetry){
        odo.update();

        Pose2D currentPose = odo.getPosition();

        // Get robot position on field
        double scale = AutoBlueConstants.MODEL_POS_SCALE;
        double robotX = currentPose.getX(DistanceUnit.CM) * scale + SCALAR_FACTOR;
        double robotY = currentPose.getY(DistanceUnit.CM) * scale+ SCALAR_FACTOR;
        double robotHeadingDegrees = Math.toDegrees(currentPose.getHeading(AngleUnit.DEGREES));

        // Normalize robot heading to 0-360, Negate Robot Heading Degrees
        robotHeadingDegrees = (((-robotHeadingDegrees) % 360) + 360) % 360;

        // Calculate vector from robot to goal
        double deltaX = GOAL_X - robotX;
        double deltaY = GOAL_Y - robotY;
        double distanceToGoal = Math.hypot(deltaY, deltaX);

        // Calculate absolute angle to goal on the field (in degrees)
        double fieldAngleToGoalDegrees;
        if(mode == "teleop"){
            fieldAngleToGoalDegrees = Math.toDegrees(Math.atan(-deltaY / deltaX));
        } else {
            fieldAngleToGoalDegrees = Math.toDegrees(Math.atan(deltaX / deltaY));
            fieldAngleToGoalDegrees = (90+fieldAngleToGoalDegrees);
        }

        if(target == "motif"){
            fieldAngleToGoalDegrees = 179.8;
        }

        fieldAngleToGoalDegrees = ((fieldAngleToGoalDegrees % 360) + 360) % 360;

        // Calculate turret angle in USER space
        // Turret angle = (Field angle to goal) - (Robot heading) + (Turret offset)
        double turretTargetUserDegrees = fieldAngleToGoalDegrees - robotHeadingDegrees + TURRET_OFFSET_DEGREES;
        turretTargetUserDegrees = ((turretTargetUserDegrees % 360) + 360) % 360;

        // Get current turret position in USER space
        double currentTurretUserDegrees = rtp.getCurrentTurretAngle();

        // Convert target to encoder space and get safe target ticks
        double safeTargetTicks = rtp.getSafeTarget(turretTargetUserDegrees);
        rtp.targetPosition = safeTargetTicks;

        // Convert target ticks to encoder degrees for PID
        double targetEncoderDegrees = safeTargetTicks / rtp.TICKS_PER_DEGREE;
        targetEncoderDegrees = rtp.normalizeAngle(targetEncoderDegrees);

        // Get current position in encoder space for PID
        double currentEncoderDegrees = rtp.getCurrentTurretAngle();

        // Calculate power using PID in encoder space
        double power = rtp.calculateSmartPID(currentEncoderDegrees, targetEncoderDegrees);

        // Apply power
        rtp.setTurretPower(power);

        // Telemetry - show everything in USER space for clarity
        telemetry.addData("=== GOAL ===", "");
        telemetry.addData("Goal Position", "X: %.1f, Y: %.1f CM", GOAL_X, GOAL_Y);

        telemetry.addData("=== ROBOT ===", "");
        telemetry.addData("Robot Position", "X: %.1f, Y: %.1f CM", robotX, robotY);
        telemetry.addData("Robot Heading", "%.1f° (user)", robotHeadingDegrees);

        telemetry.addData("=== TURRET (User Space) ===", "");
        telemetry.addData("Field Angle to Goal", "%.1f°", fieldAngleToGoalDegrees);
        telemetry.addData("Turret Target", "%.1f°", turretTargetUserDegrees);
        telemetry.addData("Turret Current", "%.1f°", currentTurretUserDegrees);
        telemetry.addData("Error", "%.1f°", turretTargetUserDegrees - currentTurretUserDegrees);

        telemetry.addData("=== TURRET (Encoder Space) ===", "");
        telemetry.addData("Target", "%.1f°", targetEncoderDegrees);
        telemetry.addData("Current", "%.1f°", currentEncoderDegrees);
        telemetry.addData("Route", rtp.getRouteInfo(currentEncoderDegrees, targetEncoderDegrees));

        telemetry.addData("=== CONTROL ===", "");
        telemetry.addData("Power", "%.3f", power);
        //telemetry.addData("Angle Inversion", rtp.INVERT_ANGLES ? "ON (45°→315°)" : "OFF");
    }
    public void adjustOffset(double offset){
        TURRET_OFFSET_DEGREES += offset;
    }
}