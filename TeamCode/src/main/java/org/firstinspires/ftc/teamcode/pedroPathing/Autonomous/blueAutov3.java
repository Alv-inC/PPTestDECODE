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
     private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(33.6, 135.4, Math.toRadians(180));
    private PathChain firstShots;
    private PathChain firstLine;
    private PathChain hitSwitch;
    private PathChain Shots;
    private PathChain AAA;
    private PathChain secondLine;
    private PathChain thirdLine;

    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;

    private DcMotorEx intake;
    private float tiltAngle = 135;
    private double count = 0;

    private boolean hasStarted = false;
    private Servo camera;

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
                                new Pose(17, 84.407)
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

        AAA = follower.pathBuilder()
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
                                new Pose(14, 61.6)
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
                                new Pose(19, 59.528)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        }


    public void autonomousPathUpdate() {
        switch (pathState) {
    //START PATH***
            case 0:
                turret.setTargetPosition(-1500);
                flyWheel.constantShoot();
                follower.followPath(firstShots, true);
                setPathState(1);
                break;

                //FIRST SHOTS***
            case 1:
                if (!follower.isBusy()) {
                   if(flyWheel.isCurrentVelocityEnough()) {
                       intake.setPower(-1);
                       flyWheel.uppies();
                        if(pathTimer.getElapsedTimeSeconds() > 4) {
                            flyWheel.downies();
                            intake.setPower(-1);
                            follower.followPath(firstLine, true);
                            setPathState(3);
                        }
                    }
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    turret.setTargetPosition(2600);
                    follower.followPath(Shots, true);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 3) {
                        flyWheel.uppies();
                    }
                    if(pathTimer.getElapsedTimeSeconds() > 5.5) {
                        flyWheel.downies();
                        follower.followPath(secondLine, true);
                        setPathState(5);
                    }

                }
                break;
            case 5:
                if (!follower.isBusy()) {

                    follower.followPath(Shots, true);
                    setPathState(7);
                }
                break;
                //SHOOT SHOTS***
            case 6:
                if (!follower.isBusy()) {
                    flyWheel.uppies();
                        if(pathTimer.getElapsedTimeSeconds() > 2.5) {
                            flyWheel.downies();
                            follower.followPath(hitSwitch, true);
                            count += 1;
                            if(count >= 3){
                                setPathState(8);
                            }else{
                                setPathState(5);
                            }
                        }

                }
                break;

                //HIT SWITCH
            case 7:
                if (!follower.isBusy()) {
                    if(pathTimer.getElapsedTimeSeconds() > 2){
                        flyWheel.uppies();
                        if(pathTimer.getElapsedTimeSeconds() > 3.5) {
                            flyWheel.downies();
                            setPathState(6);
                        }
                    }



                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    //shoot balls
                    if(pathTimer.getElapsedTimeSeconds() > 4.5) {
                        follower.followPath(hitSwitch, true);
                        setPathState(6);
                    }

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
        camera = hardwareMap.get(Servo.class, "camera");
        limelight = new LimelightCamera(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setHigh();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        limelight.update();
        int targetTagId = 20;
//        limelight.trackTag_New(turret, targetTagId, true);

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
