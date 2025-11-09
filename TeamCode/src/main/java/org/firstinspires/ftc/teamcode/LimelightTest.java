//package org.firstinspires.ftc.teamcode;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
//import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
//
//import java.util.List;
//
//@Config
//@TeleOp(name = "Limelight: Turret Tracker (Manual Trigger)", group = "TeleOp")
//public class LimelightTest extends LinearOpMode {
//
//    // --------- DASHBOARD CONFIG ---------
//    public static int PIPELINE_INDEX = 0;
//    public static boolean ENABLE_TURRET_TRACKING = false;   // toggle manually in FTC Dashboard
//    public static double CORRECTION_GAIN = 0.7;              // how strongly the turret reacts
//    public static double DEADBAND_DEG = 0.5;                 // ignore tiny deviations
//    public static double GEAR_RATIO = 1.0;                   // turret gearing ratio
//    public static double TICKS_PER_REV = 384.5;              // motor counts per revolution
//    public static double TICKS_PER_DEG = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
//    // -----------------------------------
//
//    @Override
//    public void runOpMode() {
//        // ---- Init hardware ----
//        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        Turret turret = new Turret(hardwareMap, telemetry);
//
//        limelight.setPollRateHz(100);
//        limelight.start();
//
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//
//        telemetry.setMsTransmissionInterval(50);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//            limelight.pipelineSwitch(PIPELINE_INDEX);
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//                telemetry.addData("valid", true);
//                telemetry.addData("pipeline", result.getPipelineIndex());
//
//                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//                if (!fiducials.isEmpty()) {
//                    LLResultTypes.FiducialResult fid = fiducials.get(0);
//
//                    // ---- Tag pose relative to camera ----
//                    Pose3D tagPoseCam = fid.getTargetPoseCameraSpace();
//                    double tz = tagPoseCam.getPosition().z;
//                    double tx = tagPoseCam.getPosition().x;
//
//                    // ---- Compute horizontal offset in degrees ----
//                    double tx_deg = Math.toDegrees(Math.atan2(tx, tz));
//                    if (Math.abs(tx_deg) < DEADBAND_DEG) tx_deg = 0.0;
//
//                    telemetry.addData("Tag ID", fid.getFiducialId());
//                    telemetry.addData("tx_deg", tx_deg);
//                    telemetry.addData("tz (m)", tz);
//                    telemetry.addData("ENABLE_TURRET_TRACKING", ENABLE_TURRET_TRACKING);
//
//                    // ---- Apply correction ONLY if manually enabled ----
//                    if (ENABLE_TURRET_TRACKING) {
//                        double correctionTicks = tx_deg * TICKS_PER_DEG * CORRECTION_GAIN;
//                        double newTarget = turret.getCurrentPosition() - correctionTicks;
//                        turret.setTargetPosition(newTarget);
//                        ENABLE_TURRET_TRACKING = false;
//                    }
//
//                    turret.update();
//
//                    telemetry.addData("Turret Pos", turret.getCurrentPosition());
//                    telemetry.addData("Turret Target", Turret.targetPosition);
//                }
//            } else {
//                telemetry.addLine("No tag detected");
//            }
//
//            telemetry.update();
//            dashboardTelemetry.update();
//            sleep(20);
//        }
//    }
//}
