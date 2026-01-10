package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;

@Config
@TeleOp(name = "Turret Tuning [NEW]", group = "Tuning")
public class NewTurretTuning extends OpMode {

    private final ElapsedTime timer = new ElapsedTime();
    private TurretPLUSIntake turret;
    public static double p, i, d, targetPosition;
    private LimelightCamera limelightCamera;
    public static int targetTagId = 20;
    public static boolean trackingEnabled = false;
    private DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");

    @Override
    public void init() {
        p = 0.0002625;
        i = 0.0;
        d = 0.00001;
        targetPosition = 0;
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        limelightCamera = new LimelightCamera(hardwareMap, telemetry);
        telemetry.addLine("Turret Tuning Ready");
        telemetry.addData("Initial Position", turret.getCurrentPosition());
        telemetry.update();

        timer.reset();
    }

    @Override
    public void loop() {

        limelightCamera.update();

        TurretPLUSIntake.p = p;
        TurretPLUSIntake.i = i;
        TurretPLUSIntake.d = d;

        limelightCamera.trackTag_New(turret, targetTagId, trackingEnabled);

        turret.update();

        telemetry.addData("Position", turret.getCurrentPosition());
        telemetry.addData("Target Position", TurretPLUSIntake.targetPosition);
        telemetry.addData("Power", turret.getPow());
        telemetry.addData("Time", timer.seconds());
        telemetry.update();
    }
}

