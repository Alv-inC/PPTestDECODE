package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Flywheel Dashboard Test", group = "Test")
public class TestFlywheel extends LinearOpMode {

    public static double TEST_VELOCITY = 0;  // set from Dashboard
    public static double INTAKE_POWER = 0;  // set from Dashboard
    public static boolean RUN_FLYWHEEL = false; // toggle from Dashboard

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        flyWheel shooter = new flyWheel(hardwareMap, telemetry);
//        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
//        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready. Use Dashboard to set velocity and toggle RUN_FLYWHEEL.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (RUN_FLYWHEEL) {
                shooter.constantShootAtVelocity((int)TEST_VELOCITY);
            } else {
                shooter.constantStop();
            }
//            intake.setPower(INTAKE_POWER);


            shooter.update();

            telemetry.addData("RUN_FLYWHEEL", RUN_FLYWHEEL);
            telemetry.addData("Target Velocity", flyWheel.targetVelocity);
            telemetry.addData("Current Velocity", flyWheel.currentVelocity);
            telemetry.addData("Power", flyWheel.power);
            telemetry.update();

            dashboardTelemetry.addData("RUN_FLYWHEEL", RUN_FLYWHEEL);
            dashboardTelemetry.addData("Target Velocity", flyWheel.targetVelocity);
            dashboardTelemetry.addData("Current Velocity", flyWheel.currentVelocity);
            dashboardTelemetry.addData("Power", flyWheel.power);
            dashboardTelemetry.update();

            sleep(20);
        }
    }
}
