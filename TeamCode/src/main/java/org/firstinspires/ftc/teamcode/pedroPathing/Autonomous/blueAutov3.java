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

@Autonomous(name = "[NEW]blueAutoV3", group = "Tests")
public class blueAutov3 extends OpMode {
    private ElapsedTime TotalTime = new ElapsedTime();
     private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(33.6, 135.4, Math.toRadians(180));

    private PathChain firstLine;
    private PathChain Switch;
    private PathChain ShotsTurn;
    private PathChain ShotsNoTurn;
    private boolean isTracking;


    private PathChain secondLine;
    private PathChain thirdLine;

    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;

    private DcMotorEx intake;
    private float tiltAngle = 150;
    private int switchCycles = 0;
    private static final int MAX_SWITCH_CYCLES = 2;

    private boolean hasStarted = false;
    private Servo camera;

    public void buildPaths() {
        ShotsNoTurn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(33.600, 135.400),

                                new Pose(56.410, 78.455)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        secondLine = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.410, 78.455),
                                new Pose(53.065, 58.160),
                                new Pose(20.525, 59.879)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();


        Switch = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.410, 78.455),
                                new Pose(32.097, 60.135),
                                new Pose(14.856, 61.538)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(tiltAngle))

                .build();

        ShotsTurn = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.856, 61.538),

                                new Pose(56.410, 78.455)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(tiltAngle))

                .build();

        firstLine = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.410, 78.455),
                                new Pose(40.197, 84.972),
                                new Pose(15.020, 84.730)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


        thirdLine = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(56.410, 78.455),
                                new Pose(59.281, 31.861),
                                new Pose(14.317, 35.452)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
    //START PATH***
            case 0: // Start → Shots (no turn)
                follower.followPath(ShotsNoTurn, true);
                setPathState(1);
                break;

            case 1: // → Second Line
                if (!follower.isBusy()) {
                    follower.followPath(secondLine, true);
                    setPathState(2);
                }
                break;

            case 2: // → Shots (no turn)
                if (!follower.isBusy()) {
                    follower.followPath(ShotsNoTurn, true);
                    setPathState(3);
                }
                break;

            case 3: // → Switch
                if (!follower.isBusy()) {
                    switchCycles = 0; // reset loop
                    follower.followPath(Switch, true);
                    if(pathTimer.getElapsedTimeSeconds() > 2.5){
                        setPathState(4);
                    }
                }
                break;

            case 4: // Switch → ShotsTurn
                if (!follower.isBusy()) {
                    follower.followPath(ShotsTurn, true);
                    setPathState(5);
                }
                break;

            case 5: // Loop check
                if (!follower.isBusy()) {
                    switchCycles++;

                    if (switchCycles < MAX_SWITCH_CYCLES) {
                        // Repeat Switch → ShotsTurn
                        follower.followPath(Switch, true);
                        if(pathTimer.getElapsedTimeSeconds() > 2.5){
                            setPathState(4);
                        }
                    } else {
                        // Done looping, move on
                        follower.followPath(firstLine, true);
                        setPathState(6);
                    }
                }
                break;

            case 6: // → Shots (no turn)
                if (!follower.isBusy()) {
                    follower.followPath(ShotsNoTurn, true);
                    setPathState(7);
                }
                break;

            case 7: // → Third Line
                if (!follower.isBusy()) {
                    follower.followPath(thirdLine, true);
                    setPathState(8);
                }
                break;

            case 8: // → Final Shots
                if (!follower.isBusy()) {
                    follower.followPath(ShotsNoTurn, true);
                    setPathState(9);
                }
                teleTest.startingPose = follower.getPose();
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
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.07);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setHigh();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        limelight.update();
        int targetTagId = 20;
        if(isTracking){
            limelight.trackTag_New(turret, targetTagId, true);
        }
        turret.update();



        if (!hasStarted) {
            pathTimer.resetTimer();   // reset your timer exactly when OpMode starts
            opmodeTimer.resetTimer();
            hasStarted = true;
            pathState = 0; // ensure the FSM begins from the right state
        }
        follower.update();
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
