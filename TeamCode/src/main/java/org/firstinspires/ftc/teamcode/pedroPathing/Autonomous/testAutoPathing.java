package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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
                private DcMotorEx intake;
                private Timer pathTimer, actionTimer, opmodeTimer;
                private int pathState;

                //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
                private final Pose startPose = new Pose(56.92710280373832, 8.446728971962548, Math.toRadians(180)).mirror();
                public PathChain Path1;
                public PathChain Path2;
                public PathChain Path3;
                public PathChain Path4;
                public PathChain Path5;
                public PathChain Path6;
                public PathChain Path7;


                private boolean hasStarted = false;


                //DEFINE THE PATHS --> REPLACE POSE WITH FINAL POS LATER
                public void buildPaths() {
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
                                            new Pose(7.523, 7.598).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();

                    Path3 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(7.523, 7.598).mirror(),

                                            new Pose(56.280, 15.224).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();

                    Path4 = follower.pathBuilder().addPath(
                                    new BezierCurve(
                                            new Pose(56.280, 15.224).mirror(),
                                            new Pose(58.381, 39.654).mirror(),
                                            new Pose(0.000, 48.220).mirror(),
                                            new Pose(8.523, 13.234).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();

                    Path5 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(8.523, 13.234).mirror(),

                                            new Pose(56.318, 15.514).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();

                    Path6 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(56.318, 15.514).mirror(),

                                            new Pose(7.981, 9.159).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();

                    Path7 = follower.pathBuilder().addPath(
                                    new BezierLine(
                                            new Pose(7.981, 9.159).mirror(),

                                            new Pose(55.738, 15.944).mirror()
                                    )
                            ).setConstantHeadingInterpolation(Math.toRadians(0))

                            .build();
                }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                if(!follower.isBusy()) {
                    intake.setPower(-1);
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
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(Path6, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(Path7, true);
                    setPathState(8);
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
        //motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
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
