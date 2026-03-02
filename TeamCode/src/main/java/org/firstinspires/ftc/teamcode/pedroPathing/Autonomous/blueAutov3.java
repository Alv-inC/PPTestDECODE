package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Camera_Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.New_Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning.breakBeamTest;
import org.firstinspires.ftc.teamcode.pedroPathing.teleTest;

@Configurable
@Autonomous(name = "[NEW]blueAutoV3", group = "Tests")
public class blueAutov3 extends OpMode {
    private ElapsedTime TotalTime = new ElapsedTime();
     private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;
    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(18.0325, 133, Math.toRadians(135));

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    private boolean isTracking, flag = true;

    private PathChain secondLine;
    private PathChain thirdLine;

    private PathChain shimmyDown;
    private PathChain shimmyUp;


    private LimelightCamera limelight;
    private New_Turret turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;
    private breakBeamTest bbTest;

    private DcMotorEx intake;

    private boolean intakeFull = false;
    private boolean trackRN = false;
    private float tiltAngle = 135;
    private int switchCycles = 0;
    private static final int MAX_SWITCH_CYCLES = 2;
    public static int initialPower = -950;
    private boolean hasStarted = false;
    private Camera_Servo camera;
    private double initTurretPosition = 0;
    private boolean updateEnd = false;
    public static Pose botPose;
    private static final long TAG_HOLD_MS = 200;   // 0.2s hold to ignore flicker
    private static final int NO_TAG_POWER = -1000;
    private long lastTagSeenMs = 0;
    private int lastGoodPower = NO_TAG_POWER;
    private boolean trackOffset = false;
    public static double offsetVal = 15;

    public void buildPaths() {
        // === SHOTS PATHS ===
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.544, 128.895),

                                new Pose(46.317, 99.091)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(46.317, 99.091),
                                new Pose(58.953, 58.340),
                                new Pose(17.477, 60.243)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(17.477, 60.243),
                                new Pose(40.784, 64.871),
                                new Pose(52.821, 84.059)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.821, 84.059),
                                new Pose(37.616, 64.067),
                                new Pose(10.579, 68.262)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.579, 68.262),

                                new Pose(2.330, 62.383)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(120))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(2.330, 62.383),
                                new Pose(35.425, 65.391),
                                new Pose(48.270, 85.184)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(48.270, 85.184),
                                new Pose(38.126, 84.830),
                                new Pose(22.495, 83.542)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(22.495, 83.542),

                                new Pose(52.355, 84.822)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(52.355, 84.822),
                                new Pose(59.000, 32.285),
                                new Pose(21.981, 35.785)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(21.981, 35.785),

                                new Pose(52.710, 108.748)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===============================
            // AUTO INIT + START → SHOT 1
            // ===============================
            case 0:
                trackRN = true;
                intake.setPower(-0.94);
                flyWheel.downies();
                flyWheel.constantShootAutoSlow(); // ONLY ONCE
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1: // delay BEFORE shooting Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    flyWheel.uppies(); // START SHOOTING WHILE MOVING
                    setPathState(2);
                }
                break;

            case 2: // shooting window for Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies(); // STOP SHOOTING
                    //flyWheel.constantShootAuto();turret.setTargetAngle(-55);
                    setPathState(3);
                }
                break;

            // ===============================
            // → SECOND LINE
            // ===============================
            case 3:
                if (!follower.isBusy()) {
                    flag = false;
                    hood.setAuto();
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            // ===============================
            // → SHOT 2 (MOVE FIRST)
            // ===============================
            case 4:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        intake.setPower(0);
                    }
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5: // wait for path to Shot 2 to finish
                if (!follower.isBusy()) {
                    intake.setPower(-0.94);
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(6);
                }
                break;

            case 6: // shooting window Shot f2
                if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(7);
                }
                break;

            // ===============================
            // → SWITCH (initialize loop)
            // ===============================
            case 7:
                switchCycles = 0;
                follower.followPath(Path4, true);
                setPathState(9);
                break;

            // ===============================
            // SWITCH → SHIMMY DOWN
            // ===============================
            case 8:
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            // ===============================
            // SHIMMY DOWN → SHIMMY UP
            // ===============================
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Path5, true);
                    setPathState(10);
                }
                break;

            // ===============================
            // SHIMMY UP → SHOT 3
            // ===============================
            case 10:
                if (intakeFull || pathTimer.getElapsedTimeSeconds() > 2) {
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        intake.setPower(0);
                    }
                    follower.followPath(Path6, true);
                    setPathState(11);
                }
                break;

            case 11: // wait for Shot 3 path to finish
                if (!follower.isBusy()) {
                    intake.setPower(-0.94);
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(12);
                }
                break;

            case 12: // shooting window Shot 3
                if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                    flyWheel.downies();
                    setPathState(13);
                }
                break;

            // ===============================
            // LOOP OR EXIT
            // ===============================
            case 13:
                switchCycles++;

                if (switchCycles < MAX_SWITCH_CYCLES) {
                    follower.followPath(Path4, true);
                    setPathState(9);
                } else {
                    follower.followPath(Path7, true);
                    setPathState(14);
                }
                break;

            // ===============================
            // → SHOT 4
            // ===============================
            case 14:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        intake.setPower(0);
                    }
                    flyWheel.downies();
                    follower.followPath(Path8, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    intake.setPower(-0.94);
                    flyWheel.uppies();
                    setPathState(16);
                }
                break;

            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 1.1) {
                    flyWheel.downies();
//                    teleTest.startingPose = follower.getPose();
//                    telemetryM.addData("end data", follower.getPose());
                    setPathState(17);
                }
                break;

            // ===============================
            // → THIRD LINE
            // ===============================
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(Path9, true);
                    setPathState(18);
                    updateEnd = true;
                    trackRN = false;
                }
                break;
//
//            // ===============================
//            // → SHOT 5
//            // ===============================
            case 18:
                if (!follower.isBusy()) {
                    trackOffset = true;
                   follower.followPath(Path10, true);
                    if(pathTimer.getElapsedTimeSeconds() > 1){
                        intake.setPower(0);
                    }
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1.5
                ) {
                    intake.setPower(-0.94);
                    flyWheel.uppies();
                    setPathState(20);
                }
                teleTest.startingPose = follower.getPose();
                telemetryM.addData("end data", follower.getPose());
                break;
//
//            case 19:
//                if (!follower.isBusy()) {
//                    flyWheel.uppies();
//                    setPathState(20);
//                }
//                break;
//
//            case 20:
//                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
//                    flyWheel.downies();
//                    setPathState(21);
//                }
//                break;
//
//            case 21:
//                teleTest.startingPose = follower.getPose();
//                telemetryM.addData("end data", follower.getPose());
//                break;
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flyWheel = new flyWheel(hardwareMap, telemetry);
        hood = new Hood(hardwareMap);
        bbTest = new breakBeamTest(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = new Camera_Servo(hardwareMap);
        camera.setHigh();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new New_Turret(hardwareMap, telemetry);
        hood.setLow();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        intakeFull = bbTest.isFull();
        limelight.update();
        int targetTagId = 20;
        if(!trackOffset) limelight.trackTag(turret, targetTagId, isTracking, 0);
        else limelight.trackTag(turret, targetTagId, isTracking, offsetVal);

        isTracking = limelight.tagInView();
//        if(!isTracking && !flag)//turret.setTargetAngle(-45);
        if(trackRN){
            turret.update();
        }

        if(updateEnd) {
            isTracking = false;
            turret.setTargetAngle(5);
            turret.update();
        }
        if (!hasStarted) {
            pathTimer.resetTimer();   // reset your timer exactly when OpMode starts
            opmodeTimer.resetTimer();
            hasStarted = true;
            pathState = 0; // ensure the FSM begins from the right state
        }
        follower.update();
        botPose = follower.getPose();

        long now = System.currentTimeMillis();

        if (limelight.tagInView()) {
            lastTagSeenMs = now;
            lastGoodPower = (int) limelight.getLaunchPower();
        }

        boolean tagRecentlySeen = (now - lastTagSeenMs) <= TAG_HOLD_MS;
        int targetPower = tagRecentlySeen ? lastGoodPower : NO_TAG_POWER;

        flyWheel.constantShootAtVelocity(targetPower);
        double power = limelight.getLaunchPower();
        if(limelight.tagInView() && !flag) flyWheel.setTargetVelocity(power);
        //else flyWheel.setTargetVelocity(initialPower);
        flyWheel.update();
        autonomousPathUpdate();

        //Feedback to Driver Hub for debugging
        telemetryM.addData("path state", pathState);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("ISTRACKING", isTracking);
        telemetryM.update();
    }
}
