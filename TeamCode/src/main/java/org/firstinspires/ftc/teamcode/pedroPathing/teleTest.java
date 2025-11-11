package org.firstinspires.ftc.teamcode.pedroPathing;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

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

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "testTele")
public class teleTest extends OpMode {
    private Follower follower;
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();

    private DcMotorEx intake;

    private Servo hood;

    private Turret turret;
    private flyWheel flywheel;

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
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel = new flyWheel(hardwareMap, telemetry);
        hood = hardwareMap.get(Servo.class, "hood");
        turret = new Turret(hardwareMap, telemetry);
        limelight = new LimelightCamera(hardwareMap, telemetry);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
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
        turret.update();

        int targetTagId = -1;
        if (gamepad2.left_trigger > 0.5) targetTagId = 20;
        else if (gamepad2.right_trigger > 0.5) targetTagId = 21;

        boolean trackingEnabled = (gamepad2.left_trigger > 0.5 || gamepad2.right_trigger > 0.5);

        limelight.trackTag(turret, targetTagId, trackingEnabled);
        limelight.logTelemetry();

        if (gamepad2.a && !previousButtonState2a) {
            if(!flag) {
             intake.setPower(0.8);
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
        if(gamepad2.left_bumper){
            flywheel.constantShootSlow();
        }
        if(gamepad2.right_bumper){
            flywheel.constantShoot();
        }
        if(gamepad2.x) {
            flywheel.constantStop();

        }



        if(gamepad2.dpad_down){
            intake.setPower(0);
        }


        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

                //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );

        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

//        Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            driveConstants.maxPower(0.4);
        }
        if(gamepad1.leftBumperWasPressed()){
            driveConstants.maxPower(0.8);
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
}