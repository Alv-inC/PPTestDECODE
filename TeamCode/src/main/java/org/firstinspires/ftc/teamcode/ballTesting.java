package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import java.util.function.Supplier;

@Configurable
@TeleOp(name = "ball testing", group = "TeleOp")

public class ballTesting extends LinearOpMode {
    private TelemetryManager telemetryM;
    public static int pipeline = 0;
    LimelightCamera camera;
    private Servo servo;
    TurretPLUSIntake turret;
    DcMotorEx intake;
    public static boolean trackBall, go, search, sequence, runIntake = false;
    public static double intakePower = 0;
    private Follower follower;
    Supplier<PathChain> pathChain;
    private ElapsedTime timer = new ElapsedTime();
    public void runOpMode() {
        // ---- Init hardware ----
        servo = hardwareMap.get(Servo.class, "camera");
        servo.setPosition(0.31);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        camera = new LimelightCamera(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 0, Math.toRadians(90)));
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            camera.update();
            camera.switchPipeline(pipeline);
            turret.update();
            camera.trackBall(turret, trackBall);
            follower.update();
            double[] result = camera.calculateBallPose(follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getHeading()), turret.getCurrentAngle()*1.25);
            telemetry.addData("x", result[0]);
            telemetry.addData("y", result[1]);
            telemetry.addData("h", result[2]);
            telemetry.addData("hr", Math.toRadians(result[2]));
            telemetry.addData("heading", follower.getHeading());
            pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                    .addPath(new Path(new BezierLine(follower::getPose, new Pose(result[0], result[1]))))
                    .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(result[2]), 0.8))
                    .build();
//            telemetry.addData("x", pose.getX());
//            telemetry.addData("y", pose.getX());
//            telemetry.addData("heading", pose.getHeading());
            if(runIntake) intake.setPower(intakePower);
            if(go){
                follower.followPath(pathChain.get());
                go = false;
            }
            if(search){
                turret.startScan();
                search = false;
            }
            else{

            }
            if(gamepad1.xWasPressed()){
                turret.startScan();
                if(camera.ballInView()){
                    turret.stopScan();
                    trackBall = true;
                }
            }
            if(gamepad1.yWasPressed() && trackBall){
                follower.followPath(pathChain.get());

            }
        }
    }
    public void automaticIntake(){
        turret.startScan();
        if(camera.ballInView()){
            turret.stopScan();
            trackBall = true;
            pause(1.5);
            follower.followPath(pathChain.get());
        }
    }
    private void pause(double seconds) {
        double start = timer.seconds();
        while (timer.seconds() - start < seconds) {
            // allow opmode to update
            follower.update();
            camera.trackBall(turret, trackBall);
            turret.update();
            camera.update();
        }
    }
}
