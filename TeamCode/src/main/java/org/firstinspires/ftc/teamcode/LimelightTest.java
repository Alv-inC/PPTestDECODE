package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.List;
import java.util.Map;

@Config
@TeleOp(name = "Limelight: Turret Tracker (Manual Trigger)", group = "TeleOp")
public class LimelightTest extends LinearOpMode {

    // --------- DASHBOARD CONFIG ---------
    public static int PIPELINE_INDEX = 1;
    public static boolean ENABLE_TURRET_TRACKING = false;   // toggle manually in FTC Dashboard
    public static double CORRECTION_GAIN = 1.5;              // how strongly the turret reacts
    public static double DEADBAND_DEG = 0.5;                 // ignore tiny deviations
    public static double GEAR_RATIO = 1.0;                   // turret gearing ratio
    public static double TICKS_PER_REV = 384.5;              // motor counts per revolution
    public static double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
    public static int TARGET_TAG_ID = 23;                    // Only follow this tag
    public static double TEST_VELOCITY = 0;  // set from Dashboard
    public static double INTAKE_POWER = 0;  // set from Dashboard
    public static boolean RUN_FLYWHEEL = false; // toggle from Dashboard
    // -----------------------------------

    @Override
    public void runOpMode() {   
        // ---- Init hardware ----
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        Turret turret = new Turret(hardwareMap, telemetry);
        flyWheel shooter = new flyWheel(hardwareMap, telemetry);
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight.setPollRateHz(100);
        limelight.start();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.setMsTransmissionInterval(50);

        waitForStart();

        while (opModeIsActive()) {
            if (RUN_FLYWHEEL) {
                shooter.constantShootAtVelocity((int)TEST_VELOCITY);
            } else {
                shooter.constantStop();
            }
            intake.setPower(INTAKE_POWER);


            shooter.update();

            limelight.pipelineSwitch(PIPELINE_INDEX);
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                telemetry.addData("valid", true);
                telemetry.addData("pipeline", result.getPipelineIndex());

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                if (!fiducials.isEmpty()) {
                    // Loop through all detected tags
                    for (LLResultTypes.FiducialResult fid : fiducials) {
                        int tagId = fid.getFiducialId();
                        telemetry.addData("id", tagId);
                        // ---- Only track if it's the specified tag ----
                        if (tagId == TARGET_TAG_ID) {
                            Pose3D tagPoseCam = fid.getTargetPoseCameraSpace();
                            double tz = tagPoseCam.getPosition().z;


                            double v = LimelightCamera.computeLaunchVelocity(tz);
                            telemetry.addData("target velocity", v);
                            double target_motor_v = LimelightCamera.velocityToTicksPerSecond(v);
                            telemetry.addData("target motor velocity", target_motor_v);
                            double tx = tagPoseCam.getPosition().x;

                            // Compute horizontal offset in degrees
                            double tx_deg = Math.toDegrees(Math.atan2(tx, tz));
                            if (Math.abs(tx_deg) < DEADBAND_DEG) tx_deg = 0.0;

                            telemetry.addData("Tag ID", tagId);
                            telemetry.addData("tx_deg", tx_deg);
                            telemetry.addData("tz (m)", tz);
                            telemetry.addData("ENABLE_TURRET_TRACKING", ENABLE_TURRET_TRACKING);

                            // Apply correction ONLY if manually enabled
                            if (ENABLE_TURRET_TRACKING) {
                                double correctionTicks = tx_deg * TICKS_PER_DEG * CORRECTION_GAIN;
                                double newTarget = turret.getCurrentPosition() + correctionTicks;
                                turret.setTargetPosition(newTarget);
                            }

                            turret.update();

                            telemetry.addData("Turret Pos", turret.getCurrentPosition());
                            telemetry.addData("Turret Target", Turret.targetPosition);
                            break; // Stop after handling the desired tag
                        }
                    }
                } else {
                    telemetry.addLine("No fiducial tags detected");
                }
            } else {
                telemetry.addLine("No Limelight result available");
            }

            telemetry.update();
            dashboardTelemetry.update();
            sleep(20);
        }
    }
}
