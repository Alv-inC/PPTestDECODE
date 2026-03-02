package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import android.graphics.PostProcessor;

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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.backupBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.blueAutov3;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.farBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.farRed;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.redAutov3;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Camera_Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.New_Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning.breakBeamTest;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "testTele")
public class teleTest extends OpMode {
    public static double initialTurretPosition = 0;
    private Follower follower;
    private static final Pose DEFAULT_POSE =
            new Pose(72.41025641025641, 0, 90);
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChainBlueFar, pathChainBlueClose, pathChainBluePark, pathChainBlueGate, ballPathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false, trackBall;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();
    private Camera_Servo camera;
    private TurretPLUSIntake turret;
    private flyWheel flywheel;
    private DcMotorEx intake;
    private Hood hood;
    LimelightCamera limelight;
    private double autoTurretAngle = 0;
    boolean flag = false;
    boolean previousButtonState2a = false;
    private boolean prevDpadUp = false;
    private boolean intaking = false;

    //final constants
    private final double block_open = -1;
    private final double block_close = 1;
    private final double hood_high = 0.4;
    private final double hood_mid = 0.2;
    private final double hood_low = 0;
    public static double x, y, r = 0;
    boolean trackingEnabled = true;
    private static final long TAG_HOLD_MS = 200;   // 0.2s hold to ignore flicker
    private static final int NO_TAG_POWER = -1000;
    private long lastTagSeenMs = 0;
    private int lastGoodPower = NO_TAG_POWER;

    private breakBeamTest bbTest;
    private boolean intakeFull;

    private boolean bbFlag = true;

    // Breakbeam flag re-arm logic
    private static final long BB_REARM_DELAY_MS = 2000; // set delay here (ms)
    private long bbRearmAtMs = 0;
    private boolean prevLeftBumper2 = false;

    @Override
    public void init() {
        bbTest = new breakBeamTest(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheel = new flyWheel(hardwareMap, telemetry);
        flywheel.constantStop();
        hood = new Hood(hardwareMap);
        hood.setHigh();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        limelight.switchPipeline(1);
        camera = new Camera_Servo(hardwareMap);
        camera.setHigh();
        follower = Constants.createFollower(hardwareMap);
        startingPose = (startingPose != null) ? startingPose : blueAutov3.botPose;
        startingPose = (startingPose != null) ? startingPose : backupBlue.botPose;
        startingPose = (startingPose != null) ? startingPose : farBlue.botPose;
        Pose poseToUse = (startingPose != null) ? startingPose : DEFAULT_POSE;
        telemetryM.addData("starting pose", poseToUse);
        telemetryM.update();
        flywheel.downies();
        follower.setStartingPose(poseToUse);
//        follower.update();
        pathChainBlueClose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(59, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
        pathChainBlueFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(70, 25))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(125), 0.8))
                .build();
        pathChainBluePark = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105, 25))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        pathChainBlueGate = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(16, 70))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
    } // blue 25, 60, 170 red 40, 80, 80

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        limelight.update();
        follower.update();
        telemetryM.update();

        long now = System.currentTimeMillis();

        // Re-arm bbFlag after delay (non-blocking)
        if (!bbFlag && bbRearmAtMs != 0 && now >= bbRearmAtMs) {
            bbFlag = true;
            bbRearmAtMs = 0;
        }

        intakeFull = bbTest.isFull();
        if (intakeFull && bbFlag) intake.setPower(0);

        telemetryM.addData("intake full", intakeFull);

//        boolean trackingEnabled = (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5 || gamepad1.left_trigger >0.5 || gamepad1.right_trigger > 0.5);
        if (gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5) trackingEnabled = true;
        if (gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5) trackingEnabled = false;
        turret.setTargetPosition(0);
        limelight.trackTag_New(turret, 20, trackingEnabled);

        telemetryM.addData("bot pose", follower.getPose());

        if (limelight.tagInView()) {
            lastTagSeenMs = now;
            lastGoodPower = (int) limelight.getLaunchPower();
        }

        boolean tagRecentlySeen = (now - lastTagSeenMs) <= TAG_HOLD_MS;
        int targetPower = tagRecentlySeen ? lastGoodPower : NO_TAG_POWER;

        flywheel.constantShootAtVelocity(targetPower);

        follower.update();
        double[] result = limelight.calculateBallPose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                Math.toDegrees(follower.getHeading()),
                turret.getCurrentAngle() * 1.25
        );
        telemetry.addData("x", result[0]);
        telemetry.addData("y", result[1]);
        telemetry.addData("h", result[2]);
        telemetry.addData("hr", Math.toRadians(result[2]));
        telemetry.addData("heading", follower.getHeading());
        ballPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(result[0], result[1]))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(result[2]), 0.8))
                .build();

        flywheel.update();

        limelight.logTelemetry(telemetryM);

        turret.update();

        if (gamepad2.a && !previousButtonState2a && !intakeFull) {
            if (!flag) {
                intake.setPower(0.5);
            } else {
                flag = false;
                intake.setPower(0);
            }
        }
        previousButtonState2a = gamepad2.a;

//        if (gamepad2.dpad_right & !intakeFull) {
//            intake.setPower(0.5);
//        }

        if (gamepad2.b && !intakeFull) {
            intake.setPower(-0.9);
        }
        if (gamepad2.y && !intakeFull) {
            intake.setPower(-0.4);
        }
        if (gamepad2.xWasPressed()) flywheel.downies();

        // Edge-trigger left bumper so the timer doesn't keep getting pushed while held
        boolean lb2Pressed = gamepad2.left_bumper && !prevLeftBumper2;
        prevLeftBumper2 = gamepad2.left_bumper;

        if (lb2Pressed) {
            flywheel.uppies();
            intake.setPower(-0.9);

            bbFlag = false;
            bbRearmAtMs = now + BB_REARM_DELAY_MS;
        }

        if (gamepad2.right_bumper) {
            flywheel.uppies();
            //flywheel.constantShoot();
            //pause(0.5);       // 0.5 second pause
        }

        if (gamepad1.dpad_up || gamepad2.dpad_up) turret.setTargetAngle(0 - autoTurretAngle);
        if (gamepad1.dpad_left || gamepad2.dpad_left) turret.setTargetAngle(120 - autoTurretAngle);
        if (gamepad1.dpad_right || gamepad2.dpad_right) turret.setTargetAngle(-120 - autoTurretAngle);

//        if (gamepad2.dpad_down) {
//            hood.setLow();
//        }
//        if (gamepad2.dpad_left) {
//            hood.setHigh();
//        }

        if (!automatedDrive) {
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * 0.75,
                    -gamepad1.left_stick_x * 0.85,
                    -gamepad1.right_stick_x * 0.8 * 0.6,
                    true// Robot Centric
            );
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier * 0.75,
                    -gamepad1.left_stick_x * slowModeMultiplier * 0.85,
                    -gamepad1.right_stick_x * slowModeMultiplier * 0.8 * 0.6,
                    true // Robot Centric
            );
        }

        if (gamepad1.xWasPressed()) {
            follower.followPath(pathChainBlueGate.get());
            automatedDrive = true;
        }
        if (gamepad1.yWasPressed()) {
            follower.followPath(pathChainBluePark.get());
            automatedDrive = true;
        }

        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.pausePathFollowing();
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        if (gamepad1.rightBumperWasPressed()) {
            follower.setMaxPower(0.7);
        }
        if (gamepad1.leftBumperWasPressed()) {
            follower.setMaxPower(1);
        }

        LynxModule hub = hardwareMap.get(LynxModule.class, "Control Hub");
        double hubCurrent = hub.getI2cBusCurrent(CurrentUnit.AMPS);

        telemetryM.addData("HUB CURRENT", intake.getCurrent(CurrentUnit.AMPS));
        telemetryM.addData("flywheel Current", flywheel.getFlyCurrent());
        telemetryM.addData("HUB CURRENT", hubCurrent);
        telemetry.update();
    }

    private void pause(double seconds) {
        double start = timer.seconds();
        while (timer.seconds() - start < seconds) {
            follower.update();
            flywheel.update();
        }
    }
}