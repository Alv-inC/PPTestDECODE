package org.firstinspires.ftc.teamcode.pedroPathing;

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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.backupBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.blueAutov3;
import org.firstinspires.ftc.teamcode.pedroPathing.Autonomous.farBlue;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.BreakPad;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Camera_Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning.breakBeamTest;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "without drive blue tele")
public class without_drive extends OpMode {
    public static double initialTurretPosition = 0;
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
    private static int NO_TAG_POWER = -1000;
    private long lastTagSeenMs = 0;
    private int lastGoodPower = NO_TAG_POWER;

    private breakBeamTest bbTest;
    private boolean intakeFull;

    private boolean bbFlag = true;

    // Breakbeam flag re-arm logic
    private static final long BB_REARM_DELAY_MS = 2000; // set delay here (ms)
    private long bbRearmAtMs = 0;
    private boolean prevLeftBumper2 = false;
    private int defaultTurretAngle = 0;

    private boolean lastTagInView = false;

    private boolean lastIntakeFull = false;
    private BreakPad breakPad;
    @Override
    public void init() {
        bbTest = new breakBeamTest(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheel = new flyWheel(hardwareMap, telemetry);
        flywheel.constantStop();
        hood = new Hood(hardwareMap);
        hood.setHighFar();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        limelight.switchPipeline(1);
        camera = new Camera_Servo(hardwareMap);
        camera.setHigh();
        breakPad = new BreakPad(hardwareMap);
        startingPose = (startingPose != null) ? startingPose : blueAutov3.botPose;
        startingPose = (startingPose != null) ? startingPose : backupBlue.botPose;
        startingPose = (startingPose != null) ? startingPose : farBlue.botPose;
        Pose poseToUse = (startingPose != null) ? startingPose : DEFAULT_POSE;
        telemetryM.addData("starting pose", poseToUse);

        flywheel.downies();
        gamepad2.setLedColor(1.0, 0.0, 0.0, 100000);


    } // blue 25, 60, 170 red 40, 80, 80

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        limelight.update();
        telemetryM.update();

        long now = System.currentTimeMillis();

        bbTest.update();

        // Re-arm bbFlag after delay (non-blocking)
        if (!bbFlag && bbRearmAtMs != 0 && now >= bbRearmAtMs) {
            bbFlag = true;
            bbRearmAtMs = 0;
        }

        intakeFull = bbTest.isFull();
        if(intakeFull && intakeFull != lastIntakeFull) gamepad2.rumbleBlips(1);
        lastIntakeFull = intakeFull;

        if (intakeFull && bbFlag) intake.setPower(0);

        telemetryM.addData("intake full", intakeFull);

//        boolean trackingEnabled = (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5 || gamepad1.left_trigger >0.5 || gamepad1.right_trigger > 0.5);
        if (gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5) trackingEnabled = true;
        if (gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5) trackingEnabled = false;
//        turret.setTargetPosition(defaultTurretAngle);
        limelight.trackTag_New(turret, 20, trackingEnabled);


        boolean inView = limelight.tagInView();
        if (inView) {
            lastTagSeenMs = now;
            lastGoodPower = (int) limelight.getLaunchPower();
        }

        if(inView != lastTagInView){
            if(!inView)gamepad2.setLedColor(1.0, 0.0, 0.0, 100000);
            else gamepad2.setLedColor(0.0, 1.0, 0.0, 100000);
        }

        lastTagInView = limelight.tagInView();

        boolean tagRecentlySeen = (now - lastTagSeenMs) <= TAG_HOLD_MS;
        int targetPower = tagRecentlySeen ? lastGoodPower : NO_TAG_POWER;

        flywheel.constantShootAtVelocity(targetPower);

        telemetryM.addData("calculated power", limelight.getLaunchPower());
        telemetryM.addData("tz", limelight.getTzMeters());

        telemetryM.addData("Target Velocity", flyWheel.targetVelocity);
        telemetryM.addData("Current Velocity", flyWheel.currentVelocity);
        telemetryM.addData("Power", flyWheel.power);


        telemetry.addData("calculated power", limelight.getLaunchPower());
        telemetry.addData("tz", limelight.getTzMeters());

        telemetry.addData("Target Velocity", flyWheel.targetVelocity);
        telemetry.addData("Current Velocity", flyWheel.currentVelocity);
        telemetry.addData("Power", flyWheel.power);

        telemetry.update();

        flywheel.update();

        turret.update();

        if ((gamepad2.a || gamepad2.cross)&& !previousButtonState2a && !intakeFull) {
            if (!flag) {
                intake.setPower(0.5);
            } else {
                flag = false;
                intake.setPower(0);
            }
        }
        previousButtonState2a = gamepad2.a || gamepad2.cross;

//        if (gamepad2.dpad_right & !intakeFull) {
//            intake.setPower(0.5);
//        }

        if ((gamepad2.b || gamepad2.circle) && !intakeFull) {
            intake.setPower(-0.9);
        }
//        if (gamepad2.y && !intakeFull) {
//            intake.setPower(-0.4);
//        }
        if (gamepad2.xWasPressed() ||gamepad2.squareWasPressed()){
            breakPad.setUp();
            flywheel.downies();
        }

        if(gamepad2.left_bumper){
            camera.setHigh_far();
            NO_TAG_POWER = -1500;
            hood.setHighFar();
        }
        if(gamepad2.right_bumper){
            hood.setHighFar();
            camera.setHigh();
            NO_TAG_POWER = -1000;
        }
        // Edge-trigger left bumper so the timer doesn't keep getting pushed while held
        if (gamepad2.yWasPressed() || gamepad2.triangleWasPressed()) {
            breakPad.setDown();
            flywheel.uppies();
            intake.setPower(-0.9);

            bbFlag = false;
            bbRearmAtMs = now + BB_REARM_DELAY_MS;
        }

//        if (gamepad2.right_bumper) {
//            flywheel.uppies();
//            //flywheel.constantShoot();
//            //pause(0.5);       // 0.5 second pause
//        }

//        if (gamepad1.dpad_up || gamepad2.dpad_up) turret.setTargetAngle(0 - autoTurretAngle);
//        if (gamepad1.dpad_left || gamepad2.dpad_left) turret.setTargetAngle(135 - autoTurretAngle);
//        if (gamepad1.dpad_right || gamepad2.dpad_right) turret.setTargetAngle(-135 - autoTurretAngle);
        if (gamepad1.dpad_up || gamepad2.dpad_up) turret.setTargetAngle(0);
        if (gamepad1.dpad_left || gamepad2.dpad_left) turret.setTargetAngle(45);
        if (gamepad1.dpad_right || gamepad2.dpad_right) turret.setTargetAngle(-45);

        if(gamepad2.dpad_down) intake.setPower(0);

//        if (gamepad2.dpad_down) {
//            hood.setLow();
//        }
//        if (gamepad2.dpad_left) {
//            hood.setHigh();
//        }



//        if (gamepad1.xWasPressed()) {
//            follower.followPath(pathChainBlueGate.get());
//            automatedDrive = true;
//        }
//        if (gamepad1.yWasPressed()) {
//            follower.followPath(pathChainBluePark.get());
//            automatedDrive = true;
//        }



        LynxModule hub = hardwareMap.get(LynxModule.class, "Control Hub");
        double hubCurrent = hub.getI2cBusCurrent(CurrentUnit.AMPS);

//        telemetryM.addData("HUB CURRENT", intake.getCurrent(CurrentUnit.AMPS));
//        telemetryM.addData("flywheel Current", flywheel.getFlyCurrent());
//        telemetryM.addData("HUB CURRENT", hubCurrent);
    }

//    private void pause(double seconds) {
//        double start = timer.seconds();
//        while (timer.seconds() - start < seconds) {
//            follower.update();
//            flywheel.update();
//        }
//    }
}