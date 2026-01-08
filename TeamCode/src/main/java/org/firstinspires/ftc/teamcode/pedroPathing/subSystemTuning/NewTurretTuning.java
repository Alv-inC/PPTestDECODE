package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.NewTurret;

@Config
@TeleOp(name = "Turret PID Tuner (Dashboard Only)")
public class NewTurretTuning extends LinearOpMode {

    // Dashboard tunable
    public static double targetAngle = 0; // degrees
    public static double targetTicks = 0; // degrees
    public static boolean target = false;
    public static double p = 0.00002;
    public static double i = 0.0;
    public static double d = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry t = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        NewTurret turret = new NewTurret(hardwareMap, t);
        turret.resetEncoder();
        // Start OpMode
        waitForStart();

        while (opModeIsActive()) {
            // Update PID constants from dashboard
            turret.p = p;
            turret.i = i;
            turret.d = d;

            // Set dashboard target
            if(target) turret.setTargetAngle(targetAngle);
            else turret.setTargetPosition(targetTicks);

            // Update turret output
            turret.update();

            // Telemetry for tuning
            t.addData("Target Angle", targetAngle);
            t.addData("Current Angle", turret.getCurrentAngle());
            t.addData("Encoder Ticks", turret.getCurrentPosition());
//            t.addData("Turret Power", turret.leftServo.getPower());
            t.update();
        }

        turret.stop();
    }
}
