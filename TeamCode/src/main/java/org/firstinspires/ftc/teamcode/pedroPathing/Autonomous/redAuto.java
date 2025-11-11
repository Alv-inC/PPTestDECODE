package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.intake;

//Visualizer: https://visualizer.pedropathing.com/
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
@Autonomous(name = "[OLD]redAuto", group = "Tests")
public class redAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(62,20, Math.toRadians(120)).mirror();
    private PathChain firstArtPos;
    private PathChain grabArtLine;
    private PathChain firstStartPos;

    private PathChain firstStartPosTurn;

    private PathChain secondArtPos;
    private PathChain grabArtLine2;
    private PathChain secondStartPos;
    private PathChain secondStartPosTurn;
    private PathChain outTemp;

    //subsystems
    private flyWheel flyWheel;
    private intake intake;

    private boolean hasStarted = false;


    //DEFINE THE PATHS --> REPLACE POSE WITH FINAL POS LATER
    public void buildPaths() {
        firstArtPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.524, 19.585).mirror(),
                                new Pose(62.478, 40).mirror(),
                                new Pose(35.216, 38.9).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        grabArtLine = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.216, 39).mirror(),
                        new Pose(10, 39).mirror()))
                .setTangentHeadingInterpolation()
                .build();

        /* This is line3. Another BezierLine, but reversed. */
        firstStartPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.286, 37.951).mirror(),
                        new Pose(62, 22).mirror()))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-110))
                .setReversed()
                .build();



        secondArtPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.524, 19.585).mirror(),
                                new Pose(63.696, 65.891).mirror(),
                                new Pose(35.997, 61.788).mirror()
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        grabArtLine2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.997, 62.788).mirror(),
                        new Pose(11.459, 62.788).mirror()))
                .setTangentHeadingInterpolation()
                .build();

        secondStartPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14.459, 61.788).mirror(), new Pose(62, 22).mirror()))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-110))
                .setReversed()
                .build();
        outTemp = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.216, 39).mirror(),
                        new Pose(11.286, 39).mirror()))
                .setTangentHeadingInterpolation()
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                flyWheel.constantShoot();
                if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                    intake.go();
                    if(pathTimer.getElapsedTimeSeconds() > 6) {
                        flyWheel.constantStop();
                        follower.followPath(firstArtPos);
                        intake.goSlow();
                        setPathState(1);
                    }
                }

                break;
            case 1:
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
            case 3:
                if (!follower.isBusy()) {
                    flyWheel.constantShootFasterDelay();
                    if(pathTimer.getElapsedTimeSeconds() > 4.5) {
                        intake.go();
                        if(pathTimer.getElapsedTimeSeconds() > 7) {
                            flyWheel.constantStop();
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(secondArtPos, true);
                    intake.goSlow();
                    setPathState(5);
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    follower.followPath(grabArtLine2, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(secondStartPos, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    flyWheel.constantShootFasterDelay();
                    if(pathTimer.getElapsedTimeSeconds() > 4.5) {
                        intake.go();
                        if(pathTimer.getElapsedTimeSeconds() > 7) {
                            flyWheel.constantStop();
                            //follower.followPath(firstArtPos);
                            setPathState(8);
                        }
                    }
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    follower.followPath(outTemp, true);
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
