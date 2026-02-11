package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import android.graphics.PostProcessor;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "adjustmentTest")
public class adjustmentTesting extends OpMode {
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();
    private Servo camera;
    private TurretPLUSIntake turret;
    private DcMotorEx intake;
    public static boolean turn, go, goTurret, enabled = false;
    LimelightCamera limelight;
    public static double turretAngle, targetHeading1 = 0;

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake (hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.52);

//        follower.update();


    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        telemetry.update();
        limelight.update();
        //Call this once per loop
        telemetryM.update();

        if(goTurret){
            turret.setTargetAngle(turretAngle);
            goTurret = false;
        }
        limelight.trackTag_New(turret, 20, enabled);

        turret.update();

        }

    }
