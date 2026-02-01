package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Wrapper for Limelight3A camera for tag detection and turret alignment.
 * Handles initialization, pipeline management, and per-tag tracking logic.
 */
@Config
public class LimelightCamera {

    private final Limelight3A limelight;
    private final Telemetry telemetry;

    // --- Tunable Dashboard Parameters ---
    public static int PIPELINE_INDEX = 1;
    public static double DEADBAND_DEG = 0.5;
    public static double CORRECTION_GAIN = -400;
//    public static double TICKS_PER_REV = 384.5;
//    public static double GEAR_RATIO = 1.0;
//    public static double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    public static double SMALL_GEAR_TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = (double) 125 /33; // big / small
    public static double TICKS_PER_TURRET_REV =
            SMALL_GEAR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEG =
            TICKS_PER_TURRET_REV / 360.0;
    public static double CAMERA_MOUNT_ANGLE = -26.5;
    public static double CAMERA_HEIGHT = 0.3048;
    public static double TARGET_HEIGHT = 0.08;
    // --- State ---
    private boolean validTarget = false;
    private double txDeg, balltxDeg = 0.0;
    private double tzMeters = 0.0;
    private int lastTagId = -1;
    private double ballDistance, ballLateralDistance = 0;

    public LimelightCamera(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }

    public void updatePipeline() {
        limelight.pipelineSwitch(PIPELINE_INDEX);
    }
    public void switchPipeline(int index) {
        if (PIPELINE_INDEX != index) {
            PIPELINE_INDEX = index;
            limelight.pipelineSwitch(index);
        }
    }


    /**
     * Processes the latest camera frame and updates tag data.
     */
    public void update() {
        //updatePipeline();
        LLResult result = limelight.getLatestResult();

        validTarget = false;
        if (result == null) {
            telemetry.addLine("no result");
            return;
        }
            if(PIPELINE_INDEX==1) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    telemetry.addLine("found tag");
                    validTarget = true;
                    LLResultTypes.FiducialResult fid = fiducials.get(0); // first tag by default
                    lastTagId = fid.getFiducialId();

                    Pose3D tagPoseCam = fid.getTargetPoseCameraSpace();
                    double tx = tagPoseCam.getPosition().x;
                    double tz = tagPoseCam.getPosition().z;
                    tzMeters = tz;

                    txDeg = Math.toDegrees(Math.atan2(tx, tz));
                    telemetry.addData("txDeg", txDeg);
                    if (Math.abs(txDeg) < DEADBAND_DEG) txDeg = 0.0;
                }
            }
            else if(PIPELINE_INDEX==0 || PIPELINE_INDEX==3){
                telemetry.addLine("hi");
                List<LLResultTypes.ColorResult> contours = result.getColorResults();
                if(!contours.isEmpty()){
                    telemetry.addLine("found ball");
                    validTarget = true;
                    LLResultTypes.ColorResult target = contours.get(0);
                    balltxDeg = target.getTargetXDegrees();
                    double ty = target.getTargetYDegrees();
                    telemetry.addData("tydegrees", ty);
// Inverted ty → subtract instead of add
                    double angle = Math.toRadians(CAMERA_MOUNT_ANGLE - ty);
                    double distance =
                            (TARGET_HEIGHT - CAMERA_HEIGHT) / Math.tan(angle);
                    ballDistance = distance;
                    telemetry.addData("ball distance", distance);
                    ballLateralDistance = ballDistance * Math.tan(Math.toRadians(balltxDeg));
                    telemetry.addData("ball lateral distance", ballLateralDistance);
                }
            }
    }

    /**
     * Performs turret correction only if the target tag matches and trigger logic allows.
     */
    public void trackTag(TurretPLUSIntake turret, int targetTagId, boolean enabled) {
        if (!validTarget || lastTagId != targetTagId || targetTagId == -1 || !enabled) return;

        double correctionTicks = txDeg * TICKS_PER_DEG;
        double newTarget = turret.getCurrentPosition() + correctionTicks;
        turret.setTargetPosition(newTarget);
    }
    public void trackTag_New(TurretPLUSIntake turret, int targetTagId, boolean enabled) {
        telemetry.addData("target id", targetTagId);
        if (!validTarget || lastTagId != targetTagId || targetTagId == -1 || !enabled) return;

        double correctionTicks = txDeg * TICKS_PER_DEG;
        double newTarget = turret.getCurrentPosition() + correctionTicks;
        telemetry.addData("newTarget", newTarget + CORRECTION_GAIN);
        turret.setTargetPosition(newTarget + CORRECTION_GAIN);
    }
    public void trackBall(TurretPLUSIntake turret, boolean enabled) {
        if (!enabled) return;
        double correctionTicks = balltxDeg * TICKS_PER_DEG;
        double newTarget = turret.getCurrentPosition() + correctionTicks;
        telemetry.addData("newTarget", newTarget + CORRECTION_GAIN);
        turret.setTargetPosition(newTarget + CORRECTION_GAIN);
    }
    /**
     * Converts a target measurement in the CAMERA frame into FIELD (absolute) XY.
     *
     * Coordinate conventions (you must keep these consistent):
     * - Robot pose: (robotXField, robotYField) in FIELD UNITS, robotHeadingDeg in degrees.
     * - Robot heading: 0 deg points along +X field axis, positive CCW (standard math).
     * - Camera frame input: xForwardMeters is forward from camera, yLeftMeters is left of camera.
     * - turretHeadingDeg: camera/turret heading relative to FIELD +X axis (absolute).
     *   If you only have turret angle relative to robot, compute:
     *     turretHeadingDeg = robotHeadingDeg + turretAngleRelativeDeg
     *
     * Returns: double[]{targetXField, targetYField}
     */
    public double[] calculateBallPose(
            double robotXField,
            double robotYField,
            double robotHeadingDeg,
            double turretAngleRelativeDeg
    ) {
        double turretHeadingDeg = robotHeadingDeg + turretAngleRelativeDeg;
        double fieldUnitsPerMeter = 39.3700787;
        // 1) Convert camera-measured target XY to FIELD units (still in camera frame)
        double xCam = ballDistance * fieldUnitsPerMeter;
        double yCam = ballDistance   * fieldUnitsPerMeter;

        double cameraOffsetXForwardMeters = 0; // camera position on robot (forward +)
        double cameraOffsetYLeftMeters = 0;     // camera position on robot (left +)
        // 2) Convert camera mounting offset to FIELD units (robot frame)
        double xOff = cameraOffsetXForwardMeters * fieldUnitsPerMeter;
        double yOff = cameraOffsetYLeftMeters * fieldUnitsPerMeter;

        // 3) Rotate camera offset from ROBOT frame -> FIELD frame using robot heading
        double rRad = Math.toRadians(robotHeadingDeg);
        double cosR = Math.cos(rRad);
        double sinR = Math.sin(rRad);

        double xOffField = xOff * cosR - yOff * sinR;
        double yOffField = xOff * sinR + yOff * cosR;

        // Camera absolute position in field coords
        double camXField = robotXField + xOffField;
        double camYField = robotYField + yOffField;

        // 4) Rotate target vector from CAMERA frame -> FIELD frame using turret (camera) heading
        double tRad = Math.toRadians(turretHeadingDeg);
        double cosT = Math.cos(tRad);
        double sinT = Math.sin(tRad);

        double xTargetFieldOffset = xCam * cosT - yCam * sinT;
        double yTargetFieldOffset = xCam * sinT + yCam * cosT;

        // 5) Translate from camera position to get absolute target position
        double targetXField = camXField + xTargetFieldOffset;
        double targetYField = camYField + yTargetFieldOffset;

//        double dx = targetXField - robotXField;
//        double dy = targetYField - robotYField;
//
//        double headingRad = Math.atan2(dy, dx);
//        double headingDeg = Math.toDegrees(headingRad);

        return new double[]{targetXField, targetYField, turretHeadingDeg};
    }

    public double getBallDistance() {
        return ballDistance;
    }
    public boolean hasValidTarget() {
        return validTarget;
    }

    public int getLastTagId() {
        return lastTagId;
    }

    public double getTxDeg() {
        return txDeg;
    }

    public double getTzMeters() {
        return tzMeters;
    }
    public static Map<String, Double> computeOptimalAngleAndV0(
            double xGoal,                // required
            Double thetaMinDeg,          // optional
            Double thetaMaxDeg          // optional

            ) {          // optional

        // Default values
        double yGoalVal = 1.15;          // meters, example target height
        double hBotVal = 0.33;             // meters, shooter height
        double gVal = 9.81;                     // m/s^2
        double thetaMinVal = (thetaMinDeg != null) ? thetaMinDeg : 20.0;  // degrees
        double thetaMaxVal = (thetaMaxDeg != null) ? thetaMaxDeg : 60.0;  // degrees
        int nSamplesVal = 1;

        if (xGoal <= 0) {
            throw new IllegalArgumentException("xGoal must be > 0 (horizontal distance).");
        }

        double dy = yGoalVal - hBotVal;
        double[] thetas = new double[nSamplesVal];
        double[] v0Vals = new double[nSamplesVal];
        Arrays.fill(v0Vals, Double.NaN);

        double thetaMinRad = Math.toRadians(thetaMinVal);
        double thetaMaxRad = Math.toRadians(thetaMaxVal);

        for (int i = 0; i < nSamplesVal; i++) {
            double th = thetaMinRad + (thetaMaxRad - thetaMinRad) * i / (nSamplesVal - 1);
            double cosT = Math.cos(th);
            double tanT = Math.tan(th);
            double denom = xGoal * tanT - dy;

            if (denom <= 0 || cosT == 0) continue;

            double v0Sq = (gVal * xGoal * xGoal) / (2 * cosT * cosT * denom);
            if (v0Sq <= 0) continue;

            v0Vals[i] = Math.sqrt(v0Sq);
            thetas[i] = th;
        }

        // Find minimum v0
        double minV0 = Double.POSITIVE_INFINITY;
        double thetaOpt = 0;
        for (int i = 0; i < nSamplesVal; i++) {
            if (!Double.isNaN(v0Vals[i]) && v0Vals[i] < minV0) {
                minV0 = v0Vals[i];
                thetaOpt = thetas[i];
            }
        }

        if (minV0 == Double.POSITIVE_INFINITY) return null;

        double v0Opt = minV0;
        double thetaOptDeg = Math.toDegrees(thetaOpt);

        // Analytical alternative angles for the same v0
        double v0_2 = v0Opt * v0Opt;
        double discriminant = v0_2 * v0_2 - gVal * (gVal * xGoal * xGoal + 2 * dy * v0_2);

        List<Double> altAngles = new ArrayList<>();
        if (discriminant >= 0) {
            double sqrtD = Math.sqrt(discriminant);
            double tan1 = (v0_2 + sqrtD) / (gVal * xGoal);
            double tan2 = (v0_2 - sqrtD) / (gVal * xGoal);

            for (double tVal : new double[]{tan1, tan2}) {
                if (Double.isFinite(tVal)) {
                    double ang = Math.toDegrees(Math.atan(tVal));
                    if (ang < 0) ang += 180.0;
                    altAngles.add(ang);
                }
            }
        } else {
            altAngles.add(thetaOptDeg);
        }

        Map<String, Double> result = new LinkedHashMap<>();
        result.put("theta_opt_deg", thetaOptDeg);
        result.put("v0_opt_m_s", v0Opt);
        if (!altAngles.isEmpty()) result.put("alt_angle_1_deg", altAngles.get(0));
        if (altAngles.size() > 1) result.put("alt_angle_2_deg", altAngles.get(1));

        return result;
    }
    public static double velocityToTicksPerSecond(double velocity) {
        // Regression equation: motor ticks/s = 290 * v_target - 58
        return 290.0 * velocity - 58.0;
    }
    public static double computeLaunchVelocity(double xGoal) {

        // Constants
        double yGoal = 1.15;     // target height (meters)
        double hBot = 0.33;      // shooter height (meters)
        double g = 9.81;         // gravity (m/s^2)
        double thetaDeg = 50.0;  // fixed launch angle
        double thetaRad = Math.toRadians(thetaDeg);

        if (xGoal <= 0) {
            throw new IllegalArgumentException("xGoal must be > 0.");
        }

        // Height difference
        double dy = yGoal - hBot;

        // Precompute trig terms
        double cosT = Math.cos(thetaRad);
        double tanT = Math.tan(thetaRad);

        // Denominator term inside projectile equation
        double inner = (2 * (dy + xGoal * tanT)) / g;
        if (inner <= 0) {
            throw new IllegalArgumentException("Target unreachable at fixed 50° angle.");
        }

        // Compute v0
        return xGoal / (cosT * Math.sqrt(inner));
    }

    public void logTelemetry(TelemetryManager telemetry) {
        telemetry.addData("Tag Valid", validTarget);
        telemetry.addData("Last Tag ID", lastTagId);
        telemetry.addData("tx (deg)", txDeg);
        telemetry.addData("tz (m)", tzMeters);
        telemetry.update();
    }
}
