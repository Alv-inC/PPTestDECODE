package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
@TeleOp(name = "Flywheel Dashboard Test", group = "Test")
public class TestFlywheel extends LinearOpMode {

    public static double TEST_VELOCITY, ph = 0;  // set from Dashboard
    public static double INTAKE_POWER = 0;  // set from Dashboard
    public static boolean RUN_FLYWHEEL, uppies = false; // toggle from Dashboard
    public static boolean RUN_POWER, enabled = false;
    public static double cameraPosition = 0.52;
    public static double hoodPosition = 0.54;

    private Servo camera;
    private Servo hood;
    private TelemetryManager dashboardTelemetry;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboardTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(cameraPosition);
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        flyWheel shooter = new flyWheel(hardwareMap, telemetry);
        shooter.downies();
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
        LimelightCamera limelight = new LimelightCamera(hardwareMap, telemetry);
        TurretPLUSIntake turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        telemetry.addLine("Ready. Use Dashboard to set velocity and toggle RUN_FLYWHEEL.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            camera.setPosition(cameraPosition);
            hood.setPosition(hoodPosition);
            limelight.update();
            turret.update();
            limelight.trackTag_New(turret, 20, enabled);
            shooter.constantShootAtVelocity((int)TEST_VELOCITY);
            shooter.update();

            if(uppies){
                uppies = false;
                shooter.uppies();
            }
            else shooter.downies();
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

            //sleep(20);
        }
    }
}
