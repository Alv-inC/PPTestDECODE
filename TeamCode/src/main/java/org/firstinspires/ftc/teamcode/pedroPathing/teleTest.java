package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import android.graphics.PostProcessor;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "testTele")
public class teleTest extends OpMode {
    public static double initialTurretPosition = 0;
    private Follower follower;
    private static final Pose DEFAULT_POSE =
            new Pose(72.41025641025641, 0, 90);
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive = false;
    private Supplier<PathChain> pathChainBlueFar, pathChainBlueClose, pathChainBluePark, pathChainBlueGate, ballPathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false, trackBall;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();
    private Servo camera;
    private TurretPLUSIntake turret;
    private flyWheel flywheel;
    private DcMotorEx intake;
    private Hood hood;
    LimelightCamera limelight;
    private double autoTurretAngle = 0;
    boolean flag = false;
    boolean previousButtonState2a = false;
    private boolean prevDpadUp = false;
    private boolean intaking = false;

    //final constants
    private final double block_open = -1;
    private final double block_close = 1;
    private final double hood_high = 0.4;
    private final double hood_mid = 0.2;
    private final double hood_low = 0;
    public static double x, y, r = 0;

    @Override
    public void init() {

        //delete later prob
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        flywheel = new flyWheel(hardwareMap, telemetry);
        flywheel.constantStop();
        hood = new Hood(hardwareMap);
        hood.setHigh();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0.65);
        follower = Constants.createFollower(hardwareMap);
        Pose poseToUse = (startingPose != null) ? startingPose : DEFAULT_POSE;
        telemetryM.addData("starting pose", poseToUse);
        telemetryM.update();
        follower.setStartingPose(poseToUse);
//        follower.update();
        pathChainBlueClose = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(59, 95))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(135), 0.8))
                .build();
        pathChainBlueFar = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(70, 25))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(125), 0.8))
                .build();
        pathChainBluePark = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(105, 25))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
                .build();
        pathChainBlueGate = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(16, 70))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
    } // blue 25, 60, 170 red 40, 80, 80

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        limelight.update();
        //Call this once per loop
        follower.update();
        telemetryM.update();

        boolean trackingEnabled = (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5);

        telemetryM.addData("bot pose", follower.getPose());
        if(limelight.tagInView()) flywheel.constantShootAtVelocity((int)limelight.getLaunchPower());
        else flywheel.constantShootAtVelocity(-1000);
//removed reset to 0
        limelight.trackBall(turret, trackBall);
        follower.update();
        double[] result = limelight.calculateBallPose(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()), turret.getCurrentAngle()*1.25);
        telemetry.addData("x", result[0]);
        telemetry.addData("y", result[1]);
        telemetry.addData("h", result[2]);
        telemetry.addData("hr", Math.toRadians(result[2]));
        telemetry.addData("heading", follower.getHeading());
        ballPathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(result[0], result[1]))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(result[2]), 0.8))
                .build();
       // if(gamepad1.left_bumper) camera.setPosition(0.26);
        if(gamepad1.right_bumper) camera.setPosition(0.6);

        int targetTagId = -1;
        if (gamepad2.left_trigger > 0.5 || gamepad1.left_trigger > 0.5) {
            limelight.switchPipeline(1);
            targetTagId = 20;
            limelight.trackTag_New(turret, targetTagId, trackingEnabled);

        }
        else if (gamepad2.right_trigger > 0.5 || gamepad1.right_trigger > 0.5) {
            limelight.switchPipeline(1);
            targetTagId = 24;
            limelight.trackTag_New(turret, targetTagId, trackingEnabled);

        }

        flywheel.update();
//        if(gamepad1.xWasPressed()){
//            limelight.switchPipeline(0);
//            turret.startScan();
//            if(limelight.ballInView()){
//                turret.stopScan();
//                trackBall = true;
//            }
//        }
//        if(gamepad1.bWasPressed()){
//            turret.stopScan();
//            trackBall = false;
//        }
//        if(gamepad1.yWasPressed() && trackBall){
//            automatedDrive = true;
//            follower.followPath(ballPathChain.get());
//        }

        if (gamepad2.dpad_up) turret.setTargetAngle(0-autoTurretAngle);

        limelight.logTelemetry(telemetryM);

        turret.update();

        if (gamepad2.a && !previousButtonState2a) {
            if(!flag) {
                intake.setPower(0.5);
            }
            else{
                flag = false;
                intake.setPower(0);
            }
        }
        previousButtonState2a = gamepad2.a;

        if(gamepad2.dpad_right){
            intake.setPower(0.5);
        }

        if(gamepad2.b){
            intake.setPower(-1);
        }
        if(gamepad2.y){
            intake.setPower(-0.4);
        }
        if(gamepad2.xWasPressed()) flywheel.downies();
        if(gamepad2.left_bumper){
            flywheel.uppies();
            //flywheel.constantShootSlow();
            //pause(0.5);       // 0.5 second pause
        }
        if(gamepad2.right_bumper){
            flywheel.uppies();
            //flywheel.constantShoot();
            //pause(0.5);       // 0.5 second pause
        }

        if(gamepad1.dpad_up)turret.setTargetAngle(0-autoTurretAngle);
        if(gamepad1.dpad_left)turret.setTargetAngle(90-autoTurretAngle);
        if(gamepad1.dpad_right)turret.setTargetPosition(-90-autoTurretAngle);



        if(gamepad2.dpad_down){
            hood.setMid();
        }
        if(gamepad2.dpad_left){
            hood.setHigh();
        }

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true// Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier * 0.6,
                    true // Robot Centric
            );


        }

        if(gamepad1.leftStickButtonWasPressed()){
            follower.followPath(pathChainBlueClose.get());
            automatedDrive = true;
            //turret.setTargetPosition(0);
        }
        if(gamepad1.rightStickButtonWasPressed()){
            follower.followPath(pathChainBlueFar.get());
            automatedDrive = true;
            //turret.setTargetPosition(0);
        }
        if(gamepad1.xWasPressed()){
            follower.followPath(pathChainBlueGate.get());
            automatedDrive = true;
            turret.setTargetPosition(0);
        }
        if(gamepad1.yWasPressed()){
            follower.followPath(pathChainBluePark.get());
            automatedDrive = true;
            turret.setTargetPosition(0);
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.pausePathFollowing();
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            follower.setMaxPower(0.7);
        }
        if(gamepad1.leftBumperWasPressed()){
            follower.setMaxPower(1);
        }

//        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
//            slowModeMultiplier += 0.25;
//        }
//
//        //Optional way to change slow mode strength
//        if (gamepad1.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
    private void pause(double seconds) {
        double start = timer.seconds();
        while (timer.seconds() - start < seconds) {
            // allow opmode to update
            follower.update();
            flywheel.update();
        }
    }

}