package org.firstinspires.ftc.teamcode.pedroPathing;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;

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
    private DcMotorEx shooterL;
    private DcMotorEx shooterR;
    private CRServo block;

    private Servo hood;

    private Turret turret;

    boolean flag = false;
    boolean previousButtonState2a = false;




    //final constants
    private final double block_open = -1;
    private final double block_close = 1;
    private final double hood_high = 0;
    private final double hood_mid = 0;
    private final double hood_low = 0;

    @Override
    public void init() {

        //delete later prob
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterRight");
        block = hardwareMap.get(CRServo.class, "block");
        hood = hardwareMap.get(Servo.class, "hood");
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = new Turret(hardwareMap, telemetry);


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
               .addPath(new Path(new BezierLine(follower::getPose, new Pose(0, 0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(0), 0.8))
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

        //Call this once per loop
        follower.update();
        telemetryM.update();


        if (gamepad1.a && !previousButtonState2a) {
            if(!flag) {
             intake.setPower(0.8);
            }
            else{
                flag = false;
               intake.setPower(0);
            }
        }
        previousButtonState2a = gamepad1.a;

        if(gamepad1.dpad_right){
            intake.setPower(1);
        }

        if(gamepad1.b){
            intake.setPower(-1);
        }
        if(gamepad1.y){
            shooterR.setPower(1);
            shooterL.setPower(1);
            new WaitCommand(1000);
            block.setPower(block_open);
        }
        if(gamepad1.x) {
            shooterR.setPower(0);
            shooterL.setPower(0);
            block.setPower(block_close);
        }

        if(gamepad1.dpad_up){
            hood.setPosition(hood_high);
        }
        if(gamepad1.dpad_left){
            hood.setPosition(hood_mid);
        }
        if(gamepad1.dpad_down){
            hood.setPosition(hood_low);
        }

        if(gamepad1.left_bumper){
            turret.setTargetPosition(0);
        }
        if(gamepad1.right_bumper){
            //reverse direction
            turret.reset();
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

//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }


        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
//        if (gamepad1.rightBumperWasPressed()) {
//            slowMode = !slowMode;
//        }

        //Optional way to change slow mode strength
//        if (gamepad1.xWasPressed()) {
////            slowModeMultiplier += 0.25;
//            follower.holdPoint(follower.getPose());
//        }

//        //Optional way to change slow mode strength
//        if (gamepad2.yWasPressed()) {
//            slowModeMultiplier -= 0.25;
//        }

        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}