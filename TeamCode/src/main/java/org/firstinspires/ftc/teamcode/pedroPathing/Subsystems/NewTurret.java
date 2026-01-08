package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NewTurret {

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx encoder;

    private final PIDController pid;
    private final Telemetry telemetry;

    // PID
    public static double p = 0.01;
    public static double i = 0.0;
    public static double d = 0.0;

    // Motion
    private double targetTicks = 0;

    // Gear math
    public static double SMALL_GEAR_TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = (double) 125 /33; // big / small
    public static double TICKS_PER_TURRET_REV =
            SMALL_GEAR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEGREE =
            TICKS_PER_TURRET_REV / 360.0;

    public NewTurret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftServo = hardwareMap.get(CRServo.class, "turret1");
        rightServo = hardwareMap.get(CRServo.class, "turret2");

        leftServo.setDirection(CRServo.Direction.REVERSE);

        encoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        pid = new PIDController(p, i, d);
    }

    public void update() {
        pid.setPID(p, i, d);

        double currentTicks = encoder.getCurrentPosition();
        double power = pid.calculate(currentTicks, targetTicks);

        power = clamp(power, -1.0, 1.0);

        leftServo.setPower(power);
        rightServo.setPower(power);

        telemetry.addData("Turret Pos", currentTicks);
        telemetry.addData("Turret Target", targetTicks);
        telemetry.addData("Turret Power", power);
    }

    public void setTargetPosition(double ticks) {
        targetTicks = ticks;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }
    public void setTargetAngle(double degrees) {
        targetTicks = degrees * TICKS_PER_DEGREE;
    }

    public double getCurrentAngle() {
        return getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean isClose(double toleranceTicks) {
        return Math.abs(getCurrentPosition() - targetTicks) < toleranceTicks;
    }

    public void stop() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public void resetEncoder() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetTicks = 0;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
