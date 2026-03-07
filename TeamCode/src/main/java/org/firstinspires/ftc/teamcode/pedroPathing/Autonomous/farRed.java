package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Camera_Servo;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

@Autonomous(name = "[NEW]farRED", group = "Tests")
public class farRed extends OpMode {
    private ElapsedTime TotalTime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose =    new Pose(56.927, 8.447, Math.toRadians(180)).mirror();

    private PathChain Path1;
    private PathChain Path2;
    private PathChain Path3;
    private PathChain Path4;
    private PathChain Path5;
    private PathChain Path6;
    private PathChain Path7;

    private PathChain leave;

    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;
    private boolean isTracking = true;

    private DcMotorEx intake;
    private float tiltAngle = 135;
    private int switchCycles = 0;
    private int count = 0;
    private static final int MAX_SWITCH_CYCLES = 2;

    private static final long TAG_HOLD_MS = 200;   // 0.2s hold to ignore flicker
    private static int NO_TAG_POWER = -1700;
    private long lastTagSeenMs = 0;
    private int lastGoodPower = NO_TAG_POWER;

    private boolean hasStarted = false;
    private Camera_Servo camera;
    private boolean trackRN = true;
    private boolean updateEnd = false;
    public static Pose botPose;public void buildPaths() {

        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.927, 8.447).mirror(),

                                new Pose(56.523, 14.542).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.523, 14.542).mirror(),
                                new Pose(28.921, 16.079).mirror(),
                                new Pose(7.5, 6.5).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.5, 6.5).mirror(),

                                new Pose(51.121, 14.551).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(51.121, 14.551).mirror(),
                                new Pose(58.605, 42.346).mirror(),
                                new Pose(2.467, 39.921).mirror(),
                                new Pose(13.682, 28.935).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.682, 28.935).mirror(),

                                new Pose(51.607, 13.047).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(51.607, 13.047).mirror(),

                                new Pose(7.5, 6.692).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(7.5, 6.692).mirror(),

                                new Pose(51.477, 13.028).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }


    public void autonomousPathUpdate() {

        switch (pathState) {

            // =========================
            // START → PATH1 → SHOOT
            // =========================
            case 0:
                //hood.setHighFar();
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    intake.setPower(-0.94);
                    flyWheel.uppies();
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 1) {
                    flyWheel.downies();
                    setPathState(3);
                }
                break;

            // =========================
            // PATH2 → PATH3 → SHOOT
            // =========================
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.7){
                        intake.setPower(0);
                    }
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        flyWheel.uppies();
                        if (pathTimer.getElapsedTimeSeconds() > 2.55) {
                            intake.setPower(-0.94);
                            setPathState(6);
                        }
                    }
                }
                break;

            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                    flyWheel.downies();
                    setPathState(7);
                }
                break;

            // =========================
            // LOOP SECTION (4–7)
            // =========================
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, true);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    count += 1;
                    if(pathTimer.getElapsedTimeSeconds() > 1.7){
                        intake.setPower(0);
                    }
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        flyWheel.uppies();
                        if (pathTimer.getElapsedTimeSeconds() > 2.05) {
                            intake.setPower(-0.94);
                            setPathState(10);
                        }

                    }
                }
                break;

            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                    flyWheel.downies();
                    setPathState(11);
                }
                break;

            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, true);
                    setPathState(12);
                    if(count > 1){
                        trackRN = false;
                        updateEnd = true;
                    }
                }
                break;

            case 12:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 1.7){
                        intake.setPower(0);
                    }
                    if(count > 1){
                        setPathState(100);

                    }else {
                        follower.followPath(Path7, true);
                    }
                    setPathState(13);
                }
                break;

            case 13:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        flyWheel.uppies();
                        if (pathTimer.getElapsedTimeSeconds() > 2.05) {
                            intake.setPower(-0.94);
                            setPathState(14);
                        }
                    }
                }
                break;

            case 14:
                if (pathTimer.getElapsedTimeSeconds() > 1.6) {
                    flyWheel.downies();
                    setPathState(7);
                }
                break;
            case 100:

                break;
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        flyWheel = new flyWheel(hardwareMap, telemetry);
        hood = new Hood(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = new Camera_Servo(hardwareMap);
        camera.setHigh_far();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setHighFar();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        limelight.update();
        int targetTagId = 24;
        limelight.trackTag_New(turret, targetTagId, isTracking);

        if(trackRN){
            turret.update();
        }
        if(updateEnd) {
            isTracking = false;
            turret.setTargetAngle(-65);
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
        flyWheel.update();
        autonomousPathUpdate();

        //Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
