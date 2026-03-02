package org.firstinspires.ftc.teamcode.pedroPathing.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.New_Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning.breakBeamTest;
import org.firstinspires.ftc.teamcode.pedroPathing.teleTest;

@Configurable
@Autonomous(name = "[NEW]redAutoV3", group = "Tests")
public class redAutov3 extends OpMode {
    private ElapsedTime TotalTime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;
    private TelemetryManager telemetryM;
    //MAYBE LATER PUT ALL THE POSES INSIDE A INITIALIZATION FUNCTION
    private final Pose startPose = new Pose(18.0325, 133, Math.toRadians(135)).mirror();

    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;
    public PathChain Path4;
    public PathChain Path5;
    public PathChain Path6;
    public PathChain Path7;
    public PathChain Path8;
    public PathChain Path9;
    public PathChain Path10;
    private boolean isTracking, flag = true;

    private PathChain secondLine;
    private PathChain thirdLine;

    private PathChain shimmyDown;
    private PathChain shimmyUp;


    private LimelightCamera limelight;
    private TurretPLUSIntake turret;
    //subsystems
    private flyWheel flyWheel;
    private Hood hood;
    private breakBeamTest bbTest;

    private DcMotorEx intake;

    private boolean intakeFull = false;
    private boolean trackRN = false;
    private float tiltAngle = 45;
    private int switchCycles = 0;
    private static final int MAX_SWITCH_CYCLES = 2;
    public static int initialPower = -950;
    private boolean hasStarted = false;
    private Servo camera;
    private double initTurretPosition = 0;
    private boolean updateEnd = false;
    public static Pose botPose;

    public void buildPaths() {
        // === SHOTS PATHS ===
        Path1 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(27.544, 128.895).mirror(),

                                new Pose(46.317, 95.502).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(46.317, 95.502).mirror(),
                                new Pose(53.569, 63.050).mirror(),
                                new Pose(9.178, 64.729).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(9.178, 64.729).mirror(),
                                new Pose(40.784, 66.441).mirror(),
                                new Pose(43.597, 84.508).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(43.597, 84.508).mirror(),
                                new Pose(37.616, 65.413).mirror(),
                                new Pose(10.579, 68.262).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(10.579, 68.262).mirror(),

                                new Pose(2.554, 62.832).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(30))

                .build();

        Path6 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(2.554, 62.832).mirror(),
                                new Pose(35.425, 65.391).mirror(),
                                new Pose(42.662, 90.567).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(45))

                .build();

        Path7 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(42.662, 90.567).mirror(),
                                new Pose(38.126, 84.830).mirror(),
                                new Pose(9.262, 90.495).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path8 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.262, 90.495).mirror(),

                                new Pose(38.673, 91.103).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path9 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(38.673, 91.103).mirror(),
                                new Pose(50.028, 41.481).mirror(),
                                new Pose(9.869, 43.290).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        Path10 = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(9.869, 43.290).mirror(),

                                new Pose(34.766, 128.262).mirror()
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            // ===============================
            // AUTO INIT + START → SHOT 1
            // ===============================
            case 0:
                trackRN = true;
                intake.setPower(-0.94);
                flyWheel.downies();
                flyWheel.constantShootAutoSlow(); // ONLY ONCE
                follower.followPath(Path1, true);
                setPathState(1);
                break;

            case 1: // delay BEFORE shooting Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 0.85) {
                    flyWheel.uppies(); // START SHOOTING WHILE MOVING
                    setPathState(2);
                }
                break;

            case 2: // shooting window for Shot 1
                if (pathTimer.getElapsedTimeSeconds() > 1.15) {
                    flyWheel.downies(); // STOP SHOOTING
                    //flyWheel.constantShootAuto();turret.setTargetAngle(-55);
                    setPathState(3);
                }
                break;

            // ===============================
            // → SECOND LINE
            // ===============================
            case 3:
                if (!follower.isBusy()) {
                    flag = false;
                    hood.setHigh();
                    follower.followPath(Path2, true);
                    setPathState(4);
                }
                break;

            // ===============================
            // → SHOT 2 (MOVE FIRST)
            // ===============================
            case 4:
                if (!follower.isBusy()) {

                    follower.followPath(Path3, true);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.15) {
                    flyWheel.downies(); // STOP SHOOTING
                    setPathState(7);
                }
                break;

            // ===============================
            // → SWITCH (initialize loop)
            // ===============================
            case 7:
                switchCycles = 0;
                follower.followPath(Path4, true);
                setPathState(9);
                break;

            // ===============================
            // SWITCH → SHIMMY DOWN
            // ===============================
            case 8:
                if (!follower.isBusy()) {
                    setPathState(9);
                }
                break;

            // ===============================
            // SHIMMY DOWN → SHIMMY UP
            // ===============================
            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(Path5, true);
                    setPathState(10);
                }
                break;

            // ===============================
            // SHIMMY UP → SHOT 3
            // ===============================
            case 10:
                if (intakeFull || pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(Path6, true);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.15) {
                    flyWheel.downies();
                    setPathState(13);
                }
                break;

            // ===============================
            // LOOP OR EXIT
            // ===============================
            case 13:
                switchCycles++;

                if (switchCycles < MAX_SWITCH_CYCLES) {
                    follower.followPath(Path4, true);
                    setPathState(9);
                } else {
                    follower.followPath(Path7, true);
                    setPathState(14);
                }
                break;

            // ===============================
            // → SHOT 4
            // ===============================
            case 14:
                if (!follower.isBusy()) {
                    flyWheel.downies();
                    follower.followPath(Path8, true);
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
                if (pathTimer.getElapsedTimeSeconds() > 1.15) {
                    flyWheel.downies();
//                    teleTest.startingPose = follower.getPose();
//                    telemetryM.addData("end data", follower.getPose());
                    setPathState(17);
                }
                break;

            // ===============================
            // → THIRD LINE
            // ===============================
            case 17:
                if (!follower.isBusy()) {
                    follower.followPath(Path9, true);
                    setPathState(18);
                }
                break;
//
//            // ===============================
//            // → SHOT 5
//            // ===============================
            case 18:
                if (!follower.isBusy()) {
                    follower.followPath(Path10, true);
                    setPathState(19);
                }
                break;
            case 19:
                if (pathTimer.getElapsedTimeSeconds() > 1.8) {
                    flyWheel.uppies();
                    setPathState(20);
                }
                teleTest.startingPose = follower.getPose();
                telemetryM.addData("end data", follower.getPose());
                break;
//
//            case 19:
//                if (!follower.isBusy()) {
//                    flyWheel.uppies();
//                    setPathState(20);
//                }
//                break;
//
//            case 20:
//                if (pathTimer.getElapsedTimeSeconds() > 1.2) {
//                    flyWheel.downies();
//                    setPathState(21);
//                }
//                break;
//
//            case 21:
//                teleTest.startingPose = follower.getPose();
//                telemetryM.addData("end data", follower.getPose());
//                break;
        }
    }



    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flyWheel = new flyWheel(hardwareMap, telemetry);
        hood = new Hood(hardwareMap);
        bbTest = new breakBeamTest(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.65);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        hood.setLow();
        flyWheel.downies();
    }

    @Override
    public void loop() {
        intakeFull = bbTest.isFull();
        limelight.update();
        int targetTagId = 24;
        limelight.trackTag_New(turret, targetTagId, isTracking);
        isTracking = limelight.tagInView();
//        if(!isTracking && !flag)//turret.setTargetAngle(-45);
        if(trackRN){
            turret.update();
        }
        if(updateEnd) {
            isTracking = false;
//            turret.setTargetAngle(0);
            turret.update();
        }
        if (!hasStarted) {
            pathTimer.resetTimer();   // reset your timer exactly when OpMode starts
            opmodeTimer.resetTimer();
            hasStarted = true;
            pathState = 0; // ensure the FSM begins from the right state
        }
        follower.update();
        botPose = follower.getPose();

        double power = limelight.getLaunchPower();
        if(limelight.tagInView() && !flag) flyWheel.setTargetVelocity(power);
        //else flyWheel.setTargetVelocity(initialPower);
        flyWheel.update();
        autonomousPathUpdate();

        //Feedback to Driver Hub for debugging
        telemetryM.addData("path state", pathState);
        telemetryM.addData("x", follower.getPose().getX());
        telemetryM.addData("y", follower.getPose().getY());
        telemetryM.addData("heading", follower.getPose().getHeading());
        telemetryM.addData("ISTRACKING", isTracking);
        telemetryM.update();
    }
}
