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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;

import java.util.List;
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
    private Turret turret;

    private Servo hood;

    boolean flag = false;
    boolean previousButtonState2a = false;




    //final constants
    private final double block_open = -1;
    private final double block_close = 1;
    private final double hood_high = 0.4;
    private final double hood_mid = 0.2;
    private final double hood_low = 0;
    Limelight3A limelight;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, telemetry);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);

        turret = new Turret(hardwareMap, telemetry);
        //delete later prob
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterRight");
        block = hardwareMap.get(CRServo.class, "block");
        hood = hardwareMap.get(Servo.class, "hood");
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);


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
        limelight.pipelineSwitch(1);
    }

    @Override
    public void loop() {
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        telemetry.addData("pipeline", result.getPipelineIndex());
        if (result != null && result.isValid()) {
            telemetry.addData("valid", true);
            telemetry.addData("pipeline", result.getPipelineIndex());
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (!fiducials.isEmpty()) {
                // Loop through all detected tags
                for (LLResultTypes.FiducialResult fid : fiducials) {
                    int tagId = fid.getFiducialId();
                    telemetry.addData("id", tagId);
                    // ---- Only track if it's the specified tag ----
                    int TARGET_TAG_ID = 0;
                    if(gamepad2.left_trigger>0) TARGET_TAG_ID = 20;
                    else if(gamepad2.right_trigger>0) TARGET_TAG_ID = 24;
                    if (tagId == TARGET_TAG_ID) {
                        Pose3D tagPoseCam = fid.getTargetPoseCameraSpace();
                        double tz = tagPoseCam.getPosition().z;
                        double tx = tagPoseCam.getPosition().x;

                        // Compute horizontal offset in degrees
                        double tx_deg = Math.toDegrees(Math.atan2(tx, tz));
                        if (Math.abs(tx_deg) < 0.5) tx_deg = 0.0;

                        telemetry.addData("Tag ID", tagId);
                        telemetry.addData("tx_deg", tx_deg);
                        telemetry.addData("tz (m)", tz);

                        // Apply correction ONLY if manually enabled
//                        if (ENABLE_TURRET_TRACKING) {
                            double correctionTicks = tx_deg * (384.5/360) * 1.5;
                            double newTarget = turret.getCurrentPosition() + correctionTicks;
                            turret.setTargetPosition(newTarget);
//                        }

                        turret.update();

                        telemetry.addData("Turret Pos", turret.getCurrentPosition());
                        telemetry.addData("Turret Target", Turret.targetPosition);
                        break; // Stop after handling the desired tag
                    }
                }
            } else {
                telemetry.addLine("No fiducial tags detected");
            }
        } else {
            telemetry.addLine("No Limelight result available");
        }

        //Call this once per loop
        follower.update();
        telemetryM.update();


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
        if(gamepad2.y){
            shooterR.setPower(1);
            shooterL.setPower(1);
            new WaitCommand(1000);
            block.setPower(block_open);
        }
        if(gamepad2.x) {
            shooterR.setPower(0);
            shooterL.setPower(0);
            block.setPower(block_close);
        }

        if(gamepad2.dpad_up){
            hood.setPosition(hood_high);
        }
        if(gamepad2.dpad_left){
            hood.setPosition(hood_mid);
        }
        if(gamepad2.dpad_down){
            hood.setPosition(hood_low);
        }

        if(gamepad2.left_bumper){
            turret.setTargetPosition(0);
        }
        if(gamepad2.right_bumper){
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
            //delete this later
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