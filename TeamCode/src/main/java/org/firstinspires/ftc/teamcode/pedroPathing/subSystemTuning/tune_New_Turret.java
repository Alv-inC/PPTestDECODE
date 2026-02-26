package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.New_Turret;

@Configurable
@TeleOp(name = "MotorTurret Tuner (Dashboard)", group = "Tuning")
public class tune_New_Turret extends OpMode {

    // Dashboard controls
    public static double targetAngleDeg = 0.0;
    public static boolean holdEnabled, track = false;

    // Optional safety clamp on the setpoint
    public static double minAngleDeg = -90.0;
    public static double maxAngleDeg = 90.0;

    private New_Turret turret;
    private LimelightCamera limelight;
    private TelemetryManager telemetryManager;
    private Servo camera;

    @Override
    public void init() {
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.65);
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();
        turret = new New_Turret(hardwareMap, telemetry);

        limelight = new LimelightCamera(hardwareMap, telemetry);
        limelight.switchPipeline(1);

        telemetry.addLine("Use FTC Dashboard -> Config to change:");
        telemetry.addLine("MotorTurret.p / i / d, targetAngleDeg, holdEnabled");
        telemetry.update();

    }

    @Override
    public void loop() {
        double clampedTarget = clamp(targetAngleDeg, minAngleDeg, maxAngleDeg);

        if (holdEnabled) {
            turret.setTargetAngle(clampedTarget);
        }
        limelight.update();
        limelight.trackTag(turret, 20, track);

        turret.update();

        // Extra tuning telemetry
        double currentDeg = turret.getCurrentAngle();
        telemetryManager.addData("Target (deg)", clampedTarget);
        telemetryManager.addData("Current (deg)", currentDeg);
        telemetryManager.addData("Error (deg)", clampedTarget - currentDeg);
        telemetryManager.addData("Hold Enabled", holdEnabled);
        telemetryManager.update();
    }

    @Override
    public void stop() {
        turret.stop();
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}