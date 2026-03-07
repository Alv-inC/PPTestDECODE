package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Alliance;

import smile.interpolation.BilinearInterpolation;
import smile.interpolation.Interpolation2D;

@Config
public class turretturret  {

    /* ---------------- HARDWARE ---------------- */

    private CRServo leftServo;
    private CRServo rightServo;
    private DcMotorEx encoder;
    private Telemetry telemetry;

    /* ---------------- PID ---------------- */

    public static double p = 0.025;
    public static double i = 0;
    public static double d = 0.003;

    private final PIDController controller;

    /* ---------------- GEAR MATH ---------------- */

    public static double SMALL_GEAR_TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = (double)125 / 35;
    public static double TICKS_PER_TURRET_REV = SMALL_GEAR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;

    /* ---------------- LIMITS ---------------- */

    public static double MAX_ANGLE_DEG = 115;

    /* ---------------- TURRET OFFSET ---------------- */

    public static double turretOffsetInches = -1.496;

    /* ---------------- MOTION ---------------- */

    public static double kS = 0.1;
    public static double HOLD_TOLERANCE_TICKS = 100;

    private double targetDegrees = 0;
    private double angleOffsetDegrees = 0;

    /* ---------------- HOMING ---------------- */

    public static double homedAngleDegrees = 145;
    public static double homingPower = 0.25;

    private Mode mode = Mode.OFF;

    /* ---------------- INTERPOLATION TABLES ---------------- */

    public static final Interpolation2D farTurretInterpolation =
            new BilinearInterpolation(
                    new double[]{43,71,100},
                    new double[]{6,27},
                    new double[][]{
                            {55.2,54},
                            {61,58},
                            {79,75}
                    });

    public static final Interpolation2D closeTurretInterpolation =
            new BilinearInterpolation(
                    new double[]{38,61,85},
                    new double[]{135.5,111,88,63},
                    new double[][]{
                            {1.3,2.8,27.5,37.5},
                            {3,16.5,32,45},
                            {5.5,28,41,56},
                    });

    /* ---------------- CONSTRUCTOR ---------------- */

    public turretturret(
            CRServo left,
            CRServo right,
            DcMotorEx encoder,
            TouchSensor limitSwitch,
            Telemetry telemetry){

        this.leftServo = left;
        this.rightServo = right;
        this.encoder = encoder;
        this.telemetry = telemetry;

        controller = new PIDController(p,i,d);

        resetEncoder();
    }

    /* ---------------- ANGLE ---------------- */

    private double getRawAngleDegrees(){
        return encoder.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getAngleDegrees(){
        return getRawAngleDegrees() + angleOffsetDegrees;
    }

    private void setAngleDegrees(double angle){
        angleOffsetDegrees = angle - getRawAngleDegrees();
    }

    /* ---------------- TARGET CONTROL ---------------- */

    public void setTargetDegrees(double degrees){
        targetDegrees = degrees;
        mode = Mode.POSITION;
    }
    public void setStartingAngle(double angle){
        angleOffsetDegrees = angle - getRawAngleDegrees();
    }

    /* ---------------- AUTO AIM ---------------- */

    public void aim(Pose robotPose, Alliance alliance){

        Pose turretPose = getTurretPose(robotPose);

        Pose pose = alliance == Alliance.RED
                ? turretPose
                : mirror(turretPose);

        double targetAngle;

        if(pose.getY() > 48)
            targetAngle = closeTurretInterpolation.interpolate(pose.getX(),pose.getY());
        else
            targetAngle = farTurretInterpolation.interpolate(pose.getX(),pose.getY());

        if(alliance == Alliance.BLUE)
            targetAngle = 180 - targetAngle;

        double turretTarget =
                AngleUnit.normalizeDegrees(
                        targetAngle - Math.toDegrees(robotPose.getHeading()));

        if(turretTarget < -90)
            turretTarget += 360;

        setTargetDegrees(turretTarget);

        telemetry.addData("AutoAim Target",targetAngle);
        telemetry.addData("Turret Target",turretTarget);
    }

    /* ---------------- UPDATE LOOP ---------------- */

    public void update(){

        controller.setPID(p,i,d);

        switch(mode){

            case POSITION:

                double errorDeg = targetDegrees - getAngleDegrees();
                double power = controller.calculate(getAngleDegrees(),targetDegrees);

                setPower(power);

                break;

//            case HOME:
//
//                if(limitSwitch.isPressed()){
//                    setPower(0);
//                    setAngleDegrees(homedAngleDegrees);
//                    mode = Mode.OFF;
//                }
//                else
//                    setPower(homingPower);
//
//                break;

            case OFF:

                setPower(0);

                break;
        }

        telemetry.addData("Turret Angle",getAngleDegrees());
        telemetry.addData("Turret Target",targetDegrees);
        telemetry.addData("Encoder",encoder.getCurrentPosition());
    }

    /* ---------------- HARDWARE ---------------- */

    private void setPower(double power){
        leftServo.setPower(power);
        rightServo.setPower(power);
    }

    private void resetEncoder(){
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ---------------- UTILITIES ---------------- */

    private double clamp(double v,double min,double max){
        return Math.max(min,Math.min(max,v));
    }

    private Pose mirror(Pose pose){
        return new Pose(141.5 - pose.getX(),pose.getY(),Math.PI - pose.getHeading());
    }

    private Pose getTurretPose(Pose robotPose){
        return new Pose(
                robotPose.getX() - turretOffsetInches*Math.cos(robotPose.getHeading()),
                robotPose.getY() - turretOffsetInches*Math.sin(robotPose.getHeading()),
                robotPose.getHeading()
        );
    }

    public enum Mode{
        POSITION,
        OFF,
        HOME
    }
}