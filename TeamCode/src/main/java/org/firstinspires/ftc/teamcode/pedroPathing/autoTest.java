package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

//Visualizer: https://visualizer.pedropathing.com/
@Autonomous(name = "autoTest", group = "Tests")
public class autoTest extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(62,20, Math.toRadians(120));
    private PathChain firstArtPos;
    private PathChain grabArtLine;
    private PathChain firstStartPos;

    //subsystems
    private flyWheel flyWheel;


    //DEFINE THE PATHS --> REPLACE POSE WITH FINAL POS LATER
    public void buildPaths() {
        firstArtPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.000, 20.000),
                                new Pose(63.499, 36.666),
                                new Pose(36.666, 35.437)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        grabArtLine = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(36.666, 35.437),
                        new Pose(15.158, 35.642)
                ))
                .setTangentHeadingInterpolation()
                .build();

        /* This is line3. Another BezierLine, but reversed. */
        firstStartPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.158, 35.642),
                        new Pose(61.656, 19.664)
                ))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.holdPoint(follower.getPose());
                //change later
                //flyWheel.setTargetVelocity(300);
                follower.followPath(firstArtPos);
                setPathState(1);
                break;
            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                if (!follower.isBusy()) {

                    follower.followPath(grabArtLine, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(firstStartPos, true);
                    setPathState(3);
                }
                break;
        }
    }






    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        flyWheel = new flyWheel(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {

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
