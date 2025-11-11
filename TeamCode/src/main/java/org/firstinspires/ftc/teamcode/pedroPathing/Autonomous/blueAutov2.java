package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.teleTest;

@Autonomous(name = "[NEW]blueAuto", group = "Tests")
public class blueAutov2 extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(33.6, 135.4, Math.toRadians(-180));
    private PathChain firstShots;
    private PathChain firstLine;
    private PathChain hitSwitch;
    private PathChain secondShots;
    private PathChain secondLine;
    private PathChain thirdShots;
    private PathChain thirdLine;
    private PathChain fourthShots;


    //subsystems
    private flyWheel flyWheel;
    private intake intake;

    private boolean hasStarted = false;
    private teleTest testTele;

    public void buildPaths() {

        firstShots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(44.939, 135.598),
                                new Pose(54.318, 84.602)
                        )
                )
                //.setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // FIRST LINE
        firstLine = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(54.318, 84.602),
                        new Pose(21.883, 84.407)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // HIT SWITCH
        hitSwitch = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(21.883, 84.407),
                                new Pose(23.056, 78.936),
                                new Pose(15.803, 75.419)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // SECOND SHOTS
        secondShots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.803, 75.419),
                                new Pose(60.179, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // SECOND LINE
        secondLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.179, 87.924),
                                new Pose(63.696, 55.881),
                                new Pose(20.711, 59.788)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // THIRD SHOTS
        thirdShots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.711, 59.788),
                                new Pose(59.984, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // THIRD LINE
        thirdLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(59.984, 87.924),
                                new Pose(66.041, 29.113),
                                new Pose(20.320, 35.170)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();

        // FOURTH SHOTS
        fourthShots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(20.320, 35.170),
                                new Pose(60.179, 87.924)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(-180))
                .build();
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(firstShots, true);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(firstLine, true);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(hitSwitch, true);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(secondShots, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(secondLine, true);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(thirdShots, true);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(thirdLine, true);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(fourthShots, true);
                    setPathState(8);
                }
                break;

            case 8:
                testTele.startingPose = follower.getPose();
                // END – no more paths
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
        intake = new intake(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
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
