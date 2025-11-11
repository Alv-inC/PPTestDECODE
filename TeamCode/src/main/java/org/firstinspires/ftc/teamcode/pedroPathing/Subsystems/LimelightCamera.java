package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

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
    public static double CORRECTION_GAIN = 1.5;
    public static double TICKS_PER_REV = 384.5;
    public static double GEAR_RATIO = 1.0;
    public static double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    // --- State ---
    private boolean validTarget = false;
    private double txDeg = 0.0;
    private double tzMeters = 0.0;
    private int lastTagId = -1;

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

    /**
     * Processes the latest camera frame and updates tag data.
     */
    public void update() {
        updatePipeline();
        LLResult result = limelight.getLatestResult();

        validTarget = false;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                validTarget = true;
                LLResultTypes.FiducialResult fid = fiducials.get(0); // first tag by default
                lastTagId = fid.getFiducialId();

                Pose3D tagPoseCam = fid.getTargetPoseCameraSpace();
                double tx = tagPoseCam.getPosition().x;
                double tz = tagPoseCam.getPosition().z;
                tzMeters = tz;

                txDeg = Math.toDegrees(Math.atan2(tx, tz));
                if (Math.abs(txDeg) < DEADBAND_DEG) txDeg = 0.0;
            }
        }
    }

    /**
     * Performs turret correction only if the target tag matches and trigger logic allows.
     */
    public void trackTag(Turret turret, int targetTagId, boolean enabled) {
        if (!validTarget || lastTagId != targetTagId || !enabled) return;

        double correctionTicks = txDeg * TICKS_PER_DEG * CORRECTION_GAIN;
        double newTarget = turret.getCurrentPosition() + correctionTicks;
        turret.setTargetPosition(newTarget);
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

    public void logTelemetry() {
        telemetry.addData("Tag Valid", validTarget);
        telemetry.addData("Last Tag ID", lastTagId);
        telemetry.addData("tx (deg)", txDeg);
        telemetry.addData("tz (m)", tzMeters);
        telemetry.update();
    }
}
