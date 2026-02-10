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
    private PathChain Shot1;
    private PathChain Shot2;
    private PathChain Shot3;
    private PathChain Shot4;
    private PathChain Shot5;
    private boolean isTracking = true;


    private PathChain secondLine;
    private PathChain thirdLine;

    private PathChain shimmyDown;
    private PathChain shimmyUp;


    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;

    private DcMotorEx intake;
    private float tiltAngle = 135;
    private int switchCycles = 0;
    private static final int MAX_SWITCH_CYCLES = 2;

    private boolean hasStarted = false;
    private Servo camera;

    public void buildPaths() {
        // === SHOTS PATHS ===
        Shot1 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(33.600, 135.400),
                        new Pose(56.410, 78.455)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shot2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(15.525, 59.879),
                        new Pose(51.410, 78.455)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shot3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(0, 64),
                        new Pose(54.410, 74.455)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shot4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(8.020, 84.730),
                        new Pose(54.410, 78.455)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Shot5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(7.5, 32.452),
                        new Pose(54.410, 78.455)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


// === LINE PATHS ===
        secondLine = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(51.410, 78.455),
                        new Pose(48.065, 58.160),
                        new Pose(15.525, 59.879)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        firstLine = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(54.410, 78.455),
                        new Pose(38.197, 84.972),
                        new Pose(19, 84.730)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        thirdLine = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(54.410, 78.455),
                        new Pose(54.281, 28.861),
                        new Pose(7.317, 32.452)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


// === SWITCH PATH ===
        Switch = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Pose(56.410, 77.455),
                        new Pose(32.097, 60.135),
                        new Pose(9, 63.1)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(tiltAngle))
                .build();

        // === SHIMMY PATHS (AFTER SWITCH) ===
        shimmyDown = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(9, 63.1),   // exactly Switch end
                        new Pose(3.5, 45)    // move DOWN ~3 units
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        shimmyUp = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(3.5, 48),
                        new Pose(3.5, 61)    // back to Switch end
                ))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===============================
            // AUTO INIT + START → SHOT 1
            // ===============================
            case 0:
                intake.setPower(-1);
                flyWheel.constantShootAutoSlow(); // ONLY ONCE
                follower.followPath(Shot1, true);
                setPathState(1);
                break;

            case 1: // delay BEFORE shooting Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 0.8) {
                    flyWheel.uppies(); // START SHOOTING WHILE MOVING
                    setPathState(2);
                }
                break;

            case 2: // shooting window for Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies(); // STOP SHOOTING
                    flyWheel.constantShootAuto();
                    setPathState(3);
                }
                break;

            // ===============================
            // → SECOND LINE
            // ===============================
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(secondLine, true);
                    setPathState(4);
                }
                break;

            // ===============================
            // → SHOT 2 (MOVE FIRST)
            // ===============================
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(Shot2, true);
                    setPathState(5);
                }
                break;

            case 5: // wait for path to Shot 2 to finish
                if (!follower.isBusy()) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(6);
                }
                break;

            case 6: // shooting window Shot 2
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(7);
                }
                break;

            // ===============================
            // → SWITCH (initialize loop)
            // ===============================
            case 7:
                switchCycles = 0;
                follower.followPath(Switch, true);
                setPathState(8);
                break;

            // ===============================
            // SWITCH → SHIMMY DOWN
            // ===============================
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(shimmyDown, true);
                    setPathState(9);
                }
                break;

            // ===============================
            // SHIMMY DOWN → SHIMMY UP
            // ===============================
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(shimmyUp, true);
                    setPathState(10);
                }
                break;

            // ===============================
            // SHIMMY UP → SHOT 3
            // ===============================
            case 10:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    follower.followPath(Shot3, true);
                    setPathState(11);
                }
                break;

            case 11: // wait for Shot 3 path to finish
                if (!follower.isBusy()) {
                    flyWheel.uppies(); // START SHOOTING
                    setPathState(12);
                }
                break;

            case 12: // shooting window Shot 3
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies();
                    setPathState(13);
                }
                break;

            // ===============================
            // LOOP OR EXIT
            // ===============================
            case 13:
                flyWheel.constantStop();
                switchCycles++;

                if (switchCycles < MAX_SWITCH_CYCLES) {
                    follower.followPath(Switch, true);
                    setPathState(8);
                } else {
                    follower.followPath(firstLine, true);
                    setPathState(14);
                }
                break;

            // ===============================
            // → SHOT 4
            // ===============================
            case 14:
                if (!follower.isBusy()) {
                    follower.followPath(Shot4, true);
                    setPathState(15);
                }
                break;

            case 15:
                if (!follower.isBusy()) {
                    flyWheel.uppies();
                    setPathState(16);
                }
                break;

            case 16:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies();
                    setPathState(17);
                }
                break;

            // ===============================
            // → THIRD LINE
            // ===============================
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(thirdLine, true);
                    setPathState(18);
                }
                break;

            // ===============================
            // → SHOT 5
            // ===============================
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(Shot5, true);
                    setPathState(19);
                }
                break;

            case 19:
                if (!follower.isBusy()) {
                    flyWheel.uppies();
                    setPathState(20);
                }
                break;

            case 20:
                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
                    flyWheel.downies();
                    setPathState(21);
                }
                break;

            case 21:
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
        camera.setDirection(Servo.Direction.REVERSE);
        camera.setPosition(0.55);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setLow();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        limelight.update();
        int targetTagId = 20;
        limelight.trackTag_New(turret, targetTagId, isTracking);
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
