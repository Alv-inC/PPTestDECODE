package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Flywheel Dashboard Test", group = "Test")
public class TestFlywheel extends LinearOpMode {

    public static double TEST_VELOCITY, ph = 0;  // set from Dashboard
    public static double INTAKE_POWER = 0;  // set from Dashboard
    public static boolean RUN_FLYWHEEL = false; // toggle from Dashboard
    public static boolean RUN_POWER, enabled = false;
    private Servo hood, camera;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.52);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        flyWheel shooter = new flyWheel(hardwareMap, telemetry);
        shooter.uppies();
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        LimelightCamera limelight = new LimelightCamera(hardwareMap, telemetry);
        TurretPLUSIntake turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        telemetry.addLine("Ready. Use Dashboard to set velocity and toggle RUN_FLYWHEEL.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            limelight.update();
            turret.update();
            limelight.trackTag_New(turret, 20, enabled);
            hood.setPosition(ph);
            shooter.constantShootAtVelocity((int)TEST_VELOCITY);
            shooter.update();

            if (RUN_POWER) {
                shooter.uppies();;
                shooter.fly1.setPower(-1);
                shooter.fly2.setPower(-1);
            }
            intake.setPower(INTAKE_POWER);
            TEST_VELOCITY = limelight.getLaunchPower();
            dashboardTelemetry.addData("calculated power", limelight.getLaunchPower());
            dashboardTelemetry.addData("tz", limelight.getTzMeters());
            dashboardTelemetry.update();

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
