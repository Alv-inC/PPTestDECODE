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

@Autonomous(name = "[NEW]farRED", group = "Tests")
public class farRed extends OpMode {
    private ElapsedTime TotalTime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(64.5233644859813, 8, Math.toRadians(180)).mirror();

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

    private boolean hasStarted = false;
    private Servo camera;
    private boolean trackRN = true;
    private boolean updateEnd = false;
    public static Pose botPose;
    public void buildPaths() {
        // === SHOTS PATHS ===
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.897, 2.393).mirror(),

                                new Pose(56.3, 14.075).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.3, 14.075).mirror(),
                                new Pose(46.710, 39.061).mirror(),
                                new Pose(13.477, 36.047).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.477, 36.047).mirror(),

                                new Pose(56.2, 18.916).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.2, 18.916).mirror(),
                                new Pose(32.248, 23.117).mirror(),
                                new Pose(9.850, 9.131).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.850, 9.131).mirror(),

                                new Pose(56.2, 18.206).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.2, 18.916).mirror(),
                                new Pose(32.748, 42.402).mirror(),
                                new Pose(12.467, 43.682).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.467, 43.682).mirror(),

                                new Pose(56.2, 18.916).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        leave = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(56.2, 18.916).mirror(),
                                new Pose(50, 18.196).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===============================
            // AUTO INIT + START → SHOT 1
            // ===============================
            case 0:
                intake.setPower(-1);
//                flyWheel.constantShootAuto(); // ONLY ONCE
                hood.setHigh();
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(2);
                }
                break;

            case 2: // shooting window Shot 2
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(6);
                }
                break;
            case 6: // shooting window Shot 2
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(7);
                }
                break;
            case 7: // shooting window Shot 2
                if (!follower.isBusy()) {
                    follower.followPath(Path4, true);
                    setPathState(8);
                }
                break;
            case 8: // shooting window Shot 2
                if (!follower.isBusy()) {
                    follower.followPath(Path5, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(10);
                }
                break;
            case 10: // shooting window Shot 2
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(11);
                }
                break;
            case 11: // shooting window Shot 2
                if (!follower.isBusy()) {
                    follower.followPath(Path6, true);
                    setPathState(12);
                }
                break;
            case 12: // shooting window Shot 2
                if (!follower.isBusy()) {
                    follower.followPath(Path7, true);
                    setPathState(13);
                }
                break;
            case 13:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(14);
                }
                break;
            case 14: // shooting window Shot 2
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(15);
                }
                break;
            case 15: // shooting window Shot 2
                if (!follower.isBusy()) {
                    follower.followPath(leave, true);
                    trackRN = false;
                    updateEnd = true;
                    setPathState(16);

                }
                break;
            case 16:
                if(!follower.isBusy()){
                    teleTest.startingPose = follower.getPose();
                }

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
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.65);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setLow();
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
            turret.setTargetAngle(-70);
            turret.update();
        }

        if (!hasStarted) {
            pathTimer.resetTimer();   // reset your timer exactly when OpMode starts
            opmodeTimer.resetTimer();
            hasStarted = true;
            pathState = 0; // ensure the FSM begins from the right state
        }
        double power = limelight.getLaunchPower();
        if(limelight.tagInView()) flyWheel.setTargetVelocity(power);
        follower.update();
        botPose = follower.getPose();

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
