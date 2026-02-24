package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

//Visualizer: https://visualizer.pedropathing.com/
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
@Autonomous(name = "[TESTING|AutoPathing]", group = "Tests")
public class testAutoPathing extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(27.543925233644863,128.8953271028037, Math.toRadians(135));
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;

    private boolean hasStarted = false;




    //DEFINE THE PATHS --> REPLACE POSE WITH FINAL POS LATER
    public void buildPaths() {
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.544, 128.895),

                                new Pose(54.840, 77.334)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.840, 77.334),
                                new Pose(48.859, 57.667),
                                new Pose(19.047, 59.794)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(19.047, 59.794),

                                new Pose(54.840, 77.330)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(54.840, 77.330),
                                new Pose(37.616, 64.067),
                                new Pose(13.495, 62.206)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(13.495, 62.206),

                                new Pose(11.019, 56.103)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(120))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(11.019, 56.103),

                                new Pose(54.840, 77.334)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(135))
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    follower.followPath(Path1, true);
                    setPathState(1);

                }

                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path2, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Path3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(Path4, true);
                    setPathState(4);
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(Path5, true);
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(Path6, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    setPathState(7);
                }
                break;

        }
    }
//make one case for shooting?





    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
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
        autonomousPathUpdate();

        //Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
