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
@Autonomous(name = "blueAuto", group = "Tests")
public class blueAuto extends OpMode {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(62,20, Math.toRadians(120));
    private PathChain firstArtPos;
    private PathChain grabArtLine;
    private PathChain firstStartPos;

    private PathChain firstStartPosTurn;

    private PathChain secondArtPos;
    private PathChain grabArtLine2;
    private PathChain secondStartPos;
    private PathChain secondStartPosTurn;

    //subsystems
    private flyWheel flyWheel;
    private intake intake;


    //DEFINE THE PATHS --> REPLACE POSE WITH FINAL POS LATER
    public void buildPaths() {
        firstArtPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.524, 17.585),
                                new Pose(64.478, 36.733),
                                new Pose(35.216, 35.951)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        grabArtLine = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.216, 37),
                        new Pose(11.286, 37)))
                .setTangentHeadingInterpolation()
                .build();

        /* This is line3. Another BezierLine, but reversed. */
        firstStartPos = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(13.286, 35.951),
                        new Pose(62.524, 17.585)))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-50))
                .setReversed()
                .build();



        secondArtPos = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.524, 17.585),
                                new Pose(63.696, 63.891),
                                new Pose(35.997, 59.788)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();
        grabArtLine2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.997, 60.788),
                        new Pose(11.459, 60.788)))
                .setTangentHeadingInterpolation()
                .build();

        secondStartPos = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(14.459, 59.788), new Pose(62.524, 17.585)))
                .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(-50))
                .setReversed()
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                flyWheel.constantShoot();
                if(pathTimer.getElapsedTimeSeconds() > 2) {
                    intake.go();
                    if(pathTimer.getElapsedTimeSeconds() > 5) {
                        flyWheel.constantStop();
                        follower.followPath(firstArtPos);
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
                    flyWheel.constantShoot();
                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        intake.go();
                        if(pathTimer.getElapsedTimeSeconds() > 5) {
                            flyWheel.constantStop();
                            setPathState(4);
                        }
                    }
                }
                break;

            case 4:
                if(!follower.isBusy()){
                    follower.followPath(secondArtPos, true);
                    intake.go();
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
                    flyWheel.constantShoot();
                    if(pathTimer.getElapsedTimeSeconds() > 2) {
                        intake.go();
                        if(pathTimer.getElapsedTimeSeconds() > 5) {
                            flyWheel.constantStop();
                            //follower.followPath(firstArtPos);
                            setPathState(6);
                        }
                    }
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
