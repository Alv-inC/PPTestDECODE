package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;
import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;
import static org.firstinspires.ftc.teamcode.pedroPathing.Constants.driveConstants;

import android.graphics.PostProcessor;

import com.acmerobotics.dashboard.config.Config;
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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "adjustmentTest")
public class adjustmentTesting extends OpMode {
    private Follower follower;
    private static final Pose DEFAULT_POSE =
            new Pose(72.41025641025641, 72.20512820512819, 90);
    public static Pose startingPose; //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;
    private ElapsedTime timer = new ElapsedTime();
    private Servo camera;
    private TurretPLUSIntake turret;
    private DcMotorEx intake;
    public static boolean turn, go, goTurret = false;
    LimelightCamera limelight;
    public static double turretAngle, targetHeading1 = 0;
    boolean flag = false;
    boolean previousButtonState2a = false;
    private boolean prevDpadUp = false;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake (hardwareMap, telemetry, intake);
        limelight = new LimelightCamera(hardwareMap, telemetry);
        camera = hardwareMap.get(Servo.class, "camera");
        camera.setPosition(0);
        follower = Constants.createFollower(hardwareMap);
        if (startingPose != null) {
            follower.setStartingPose(startingPose);
        } else {
            follower.setStartingPose(DEFAULT_POSE);
        }
//        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(34.46153846153846, 117.94871794871796))))
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

        if(goTurret){
            turret.setTargetAngle(turretAngle);
            goTurret = false;
        }

        limelight.logTelemetry(telemetryM);

        turret.update();


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

        double turretAngleRad = Math.toRadians(turret.getCurrentAngle());
        double currentHeading = follower.getHeading();
        telemetryM.addData("current heading", currentHeading);
        double targetHeading = normalizeRadians(currentHeading + turretAngleRad);
        telemetryM.addData("target heading", targetHeading);
        if(turn){
            follower.turnTo(targetHeading);
            turret.setTargetAngle(0);
            automatedDrive = true;
            //turn = false;
        }
        if (go) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
            go = false;
        }

//        if (automatedDrive && (gamepad.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }



        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }


}