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
    private DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intake");
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(33.6, 135.4, Math.toRadians(180));
    private PathChain firstShots;
    private PathChain firstLine;
    private PathChain hitSwitch;
    private PathChain Shots;
    private PathChain secondLine;
    private PathChain thirdLine;

    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;
    private float tiltAngle = 150;
    private double count = 0;

    private boolean hasStarted = false;
    public void buildPaths() {
// FIRST SHOTS
        firstShots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(33.600, 135.400),
                                new Pose(54.318, 84.602)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

// FIRST LINE
        firstLine = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(54.318, 84.602),
                                new Pose(18.500, 84.407)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

// SHOTS
        Shots = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18.500, 84.407),
                                new Pose(62.227, 81.984)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(tiltAngle))
                .build();

// HIT SWITCH
        hitSwitch = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.227, 81.984),
                                new Pose(31.223, 67.673),
                                new Pose(14.713, 67.183)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(tiltAngle))
                .build();


// SECOND LINE
        secondLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(62.227, 81.984),
                                new Pose(60.959, 57.797),
                                new Pose(14.674, 59.528)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        }


    public void autonomousPathUpdate() {
        switch (pathState) {
    //START PATH***
            case 0:

                follower.followPath(firstShots, true);
                setPathState(1);
                break;

                //FIRST SHOTS***
            case 1:
                if (!follower.isBusy()) {
                    if(flyWheel.isCurrentVelocityEnough()) {
                       //shoot the balls, turn it on
                        if(pathTimer.getElapsedTimeSeconds() > 6) {
                            //block the balls
                            follower.followPath(firstLine, true);
                            setPathState(3);
                        }
                    }
                }
                break;
                //GO TO SHOTS LOCATION***
            case 3:
                if (!follower.isBusy()) {

                    follower.followPath(Shots, true);
                    setPathState(4);
                }
                break;

                //SHOOT SHOTS***
            case 4:
                if (!follower.isBusy()) {
                    //unlock and shoot

                        if(pathTimer.getElapsedTimeSeconds() > 4.5) {
                            //stop shooting
                            follower.followPath(hitSwitch, true);
                            count += 1;
                            if(count >= 3){
                                setPathState(5);
                            }else{
                                setPathState(4);
                            }
                        }

                }
                break;

                //HIT SWITCH
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(secondLine, true);
                    setPathState(4);
                }
                teleTest.startingPose = follower.getPose();
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
        hood = new Hood(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
    }

    @Override
    public void loop() {
//        limelight.update();
//        int targetTagId = 20;
//        limelight.trackTag(turret, targetTagId, true);
//        turret.update();

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
