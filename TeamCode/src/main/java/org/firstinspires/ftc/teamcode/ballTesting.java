package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.LimelightCamera;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

@Configurable
@TeleOp(name = "ball testing", group = "TeleOp")

public class ballTesting extends LinearOpMode {
    private TelemetryManager telemetryM;
    public static int pipeline = 0;
    LimelightCamera camera;
    private Servo servo;
    TurretPLUSIntake turret;
    DcMotorEx intake;
    public static boolean trackBall, go = false;
    private Follower follower;
    public void runOpMode() {
        // ---- Init hardware ----
        servo = hardwareMap.get(Servo.class, "camera");
        servo.setPosition(0.15);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        camera = new LimelightCamera(hardwareMap, telemetry);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turret = new TurretPLUSIntake(hardwareMap, telemetry, intake);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72.41025641025641, 72.20512820512819, 90));
        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            camera.update();
            camera.switchPipeline(pipeline);
            turret.update();
            camera.trackBall(turret, trackBall);
            follower.update();

//            telemetry.addData("x", pose.getX());
//            telemetry.addData("y", pose.getX());
//            telemetry.addData("heading", pose.getHeading());
            if(go){
                //follower.followPath();
                go = false;
            }
        }
    }
}
