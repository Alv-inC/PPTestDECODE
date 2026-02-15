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
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.teleTest;

import java.nio.file.Paths;
@Configurable
@Autonomous(name = "[NEW]backupBlue", group = "Tests")
public class backupBlue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private TelemetryManager telemetryM;

    private final Pose startPose = new Pose(33.6, 135.4, Math.toRadians(180));

    // ===== PATH DECLARATIONS =====
    private PathChain Path1;
    private PathChain Path2;
    private PathChain Path3;
    private PathChain Path4;
    private PathChain Path5;
    private PathChain Path6;
    private PathChain Path7;
    private PathChain Path8;
    private PathChain Path9;
    private PathChain Path10;

    private int pathState;
    private boolean hasStarted = false;

    // ===== SUBSYSTEMS =====
    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    private flyWheel flyWheel;
    private Hood hood;
    private DcMotorEx intake;

    private boolean isTracking;
    private boolean trackRN = false;
    private boolean flag = true;
    private boolean updateEnd = false;

    private Servo camera;

    // =============================
    // BUILD PATHS
    // =============================
    public void buildPaths() {

        Path1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.600, 135.400),
                        new Pose(50.393, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(50.393, 84.000),
                        new Pose(17.785, 84.159)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.523, 84.159),
                        new Pose(50.393, 84.000)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path4 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.393, 84.000),
                        new Pose(59.665, 55.939),
                        new Pose(12.430, 59.710)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.430, 59.710),
                        new Pose(50.495, 84.103)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(50.495, 84.103),
                        new Pose(65.919, 29.152),
                        new Pose(12.308, 35.449)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path7 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(12.308, 35.449),
                        new Pose(50.570, 84.224)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path8 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(50.570, 84.224),
                        new Pose(50.056, 69.318)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        Path8 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(50.570, 84.224),
                                new Pose(42.355, 8.757),
                                new Pose(8.6, 9.18)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(8.6, 9.18),

                                new Pose(50.215, 84.215)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(50.215, 84.215),

                                new Pose(50.308, 66.178)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

    }


    // =============================
    // FSM
    // =============================
    public void autonomousPathUpdate() {

        switch (pathState) {

            // PATH1 — shoot while moving
            case 0:

                intake.setPower(-1);
                flyWheel.constantShootAutoSlow();

                follower.followPath(Path1, true);

                setPathState(1);
                break;

            case 1:

                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    flyWheel.uppies();
                    trackRN = true;
                }

                if (!follower.isBusy()) {

                    flyWheel.downies();

                    setPathState(2);
                }

                break;


            // SHOOT STATIONARY
            case 2:

                flyWheel.uppies();

                setPathState(3);

                break;

            case 3:

                if (pathTimer.getElapsedTimeSeconds() > 1) {

                    flyWheel.downies();

                    follower.followPath(Path2, true);

                    setPathState(4);

                }

                break;


            // RETURN LEFT → RETURN CENTER
            case 4:

                if (!follower.isBusy()) {

                    follower.followPath(Path3, true);

                    setPathState(5);

                }

                break;


            // SHOOT AGAIN
            case 5:

                if (!follower.isBusy()) {

                    flyWheel.uppies();

                    setPathState(6);

                }

                break;

            case 6:

                if (pathTimer.getElapsedTimeSeconds() > 1.2) {

                    flyWheel.downies();

                    follower.followPath(Path4, true);

                    setPathState(7);

                }

                break;


            // NEXT CYCLE
            case 7:

                if (!follower.isBusy()) {

                    follower.followPath(Path5, true);

                    setPathState(8);

                }

                break;


            case 8:

                if (!follower.isBusy()) {

                    flyWheel.uppies();

                    setPathState(9);

                }

                break;


            case 9:

                if (pathTimer.getElapsedTimeSeconds() > 1.2) {

                    flyWheel.downies();

                    follower.followPath(Path6, true);

                    setPathState(10);

                }

                break;


            case 10:

                if (!follower.isBusy()) {

                    follower.followPath(Path7, true);

                    setPathState(11);

                }

                break;


            case 11:

                if (!follower.isBusy()) {

                    flyWheel.uppies();

                    setPathState(12);

                }

                break;


            case 12:

                if (pathTimer.getElapsedTimeSeconds() > 1.2) {

                    flyWheel.downies();

                    follower.followPath(Path8, true);

                    setPathState(13);

                }

                break;
            case 13:

                if (!follower.isBusy()) {

                    follower.followPath(Path9, true);

                    setPathState(14);

                }

                break;


            case 14:

                if (!follower.isBusy()) {

                    flyWheel.uppies();

                    setPathState(15);

                }

                break;


            case 15:

                if (pathTimer.getElapsedTimeSeconds() > 1.2) {

                    flyWheel.downies();

                    follower.followPath(Path10, true);

                    setPathState(16);

                }

                break;


            case 16:

                updateEnd = true;

                break;
        }
    }

    public void setPathState(int state) {

        pathState = state;
        pathTimer.resetTimer();

    }

    // =============================
    // INIT
    // =============================
    @Override
    public void init() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        flyWheel = new flyWheel(hardwareMap, telemetry);

        hood = new Hood(hardwareMap);

        pathTimer = new Timer();

        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        buildPaths();

        limelight = new LimelightCamera(hardwareMap, telemetry);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);

        camera = hardwareMap.get(Servo.class, "camera");

        camera.setPosition(0.65);

        hood.setLow();

        flyWheel.downies();
    }


    // =============================
    // LOOP
    // =============================
    @Override
    public void loop() {

        limelight.update();

        int targetTagId = 20;

        limelight.trackTag_New(turret, targetTagId, isTracking);

        isTracking = limelight.tagInView();

        if (!isTracking && !flag)
            turret.setTargetAngle(-7);

        if (trackRN)
            turret.update();

        if (updateEnd) {

            isTracking = false;

            turret.setTargetAngle(35);

            turret.update();

        }

        if (!hasStarted) {

            pathTimer.resetTimer();

            opmodeTimer.resetTimer();

            hasStarted = true;

            pathState = 0;
        }

        follower.update();

        double power = limelight.getLaunchPower();

        if (limelight.tagInView() && !flag)
            flyWheel.setTargetVelocity(power);

        flyWheel.update();

        autonomousPathUpdate();

        telemetryM.addData("state", pathState);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());

        telemetryM.update();
    }
}