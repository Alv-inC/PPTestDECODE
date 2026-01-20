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
@TeleOp(name = "testTele2")
public class TeleTest2 extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();
    private Servo camera;
    private TurretPLUSIntake turret;
    private flyWheel flywheel;
    private DcMotorEx intake;
    private Hood hood;
    LimelightCamera limelight;

    boolean flag = false;
    boolean previousButtonState2a = false;




    //final constants
    private final double block_open = -1;
    private final double block_close = 1;
    private final double hood_high = 0.4;
    private final double hood_mid = 0.2;
    private final double hood_low = 0;

    @Override
    public void init() {

        //delete later prob

        flywheel = new flyWheel(hardwareMap, telemetry);
        flywheel.constantStop();
        hood = new Hood(hardwareMap);
        hood.setHigh();
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake (hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(33.6, 135.4, Math.toRadians(180)).mirror());
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(54.318, 84.602))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(-180), 0.8))
                .build();
    }

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
        flywheel.update();

        boolean trackingEnabled = (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5);

        int targetTagId = -1;
        if (gamepad2.left_trigger > 0.5) {
            targetTagId = 20;
            limelight.trackTag_New(turret, targetTagId, trackingEnabled);

        }
        else if (gamepad2.right_trigger > 0.5) {
            targetTagId = 24;
            limelight.trackTag_New(turret, targetTagId, trackingEnabled);

        }

        if (gamepad2.dpad_up) turret.setTargetPosition(0);
        else if(gamepad2.dpad_down) turret.setTargetPosition(0);
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
            intake.setPower(1);
        }

        if(gamepad2.b){
            intake.setPower(-1);
        }
        if(gamepad2.y){
            intake.setPower(-0.7);
        }
        if(gamepad2.left_bumper){
            flywheel.uppies();
            flywheel.constantShootSlow();
            //pause(0.5);       // 0.5 second pause

        }
        if(gamepad2.right_bumper){
            flywheel.uppies();
            flywheel.constantShoot();
            //pause(0.5);       // 0.5 second pause
        }
        if(gamepad2.x) {
            flywheel.constantStop();
        }
        if(gamepad1.dpad_up)hood.setHigh();
        if(gamepad1.dpad_left)hood.setMid();
        if(gamepad1.dpad_down)hood.setLow();


//
//        if(gamepad2.dpad_down){
//            intake.setPower(0);
//        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier * 0.6,
                    false // Robot Centric
            );

        }

        //Automated PathFollowing
//        if (gamepad1.dpad_down) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            follower.setMaxPower(0.4);
        }
        if(gamepad1.leftBumperWasPressed()){
            follower.setMaxPower(0.7);
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad1.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

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