package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.turret2;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Turret Tuning OpMode", group="Tuning")
public class TurretTuningOpMode extends LinearOpMode {

    public static double kP = 0.01;   // Adjustable in FTCDashboard
    public static double targetPosition = 1000; // Example target encoder ticks

    private turret2 turret;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Initialize turret (replace with your hardware names)
        turret = new turret2(hardwareMap, "encoderMotor", "servo1", "servo2");

        waitForStart();

        while (opModeIsActive()) {

            // Update turret target
            turret.setTargetPosition(targetPosition);

            // Update kP from dashboard
            turret.kP = kP;

            // Update turret control
            turret.update();

            // Telemetry for tuning
            telemetry.addData("Current Position", turret.getCurrentPosition());
            telemetry.addData("Target Position", targetPosition);
            telemetry.addData("kP", kP);
            telemetry.update();

            sleep(20); // Small delay to prevent overload
        }

        // Stop servos at the end
        turret.servo1.setPower(0);
        turret.servo2.setPower(0);
    }
}
