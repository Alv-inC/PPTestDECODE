package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class New_Turret {

    private DcMotorEx motor;
    private PIDController pid;
    private Telemetry telemetry;

    // PID constants
    public static double p = 0.01;
    public static double i = 0.0;
    public static double d = 0.001;
    public static double kS = 0;

    // Deadband (degrees)
    public static double POSITION_DEADBAND_DEG = 5;

    // Motion state
    private double targetTicks = 0;

    // Motor / gear math
    public static double MOTOR_TICKS_PER_REV = 384.5;
    public static double GEAR_RATIO = (double) 125 / 39;
    public static double TICKS_PER_TURRET_REV =
            MOTOR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEGREE =
            TICKS_PER_TURRET_REV / 360.0;

    // Power clamp
    public static double MAX_POWER = 0.8;

    public New_Turret(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        targetTicks = 0;

        motor = hardwareMap.get(DcMotorEx.class, "turret");

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new PIDController(p, i, d);
    }

    public void update() {
        pid.setPID(p, i, d);

        double currentTicks = motor.getCurrentPosition();
        double errorTicks = targetTicks - currentTicks;

        double deadbandTicks = POSITION_DEADBAND_DEG * TICKS_PER_DEGREE;

        double power;

        if (Math.abs(errorTicks) < deadbandTicks) {
            power = 0;
            pid.reset(); // prevents integral windup inside deadband
        } else {
            power = pid.calculate(currentTicks, targetTicks);
            power = clamp(power, -MAX_POWER, MAX_POWER);
            power = power + Math.signum(currentTicks - targetTicks) * kS;
        }

        motor.setPower(power);

        telemetry.addData("Turret Pos (deg)", getCurrentAngle());
        telemetry.addData("Turret Target (deg)", targetTicks / TICKS_PER_DEGREE);
        telemetry.addData("Turret Error (deg)", errorTicks / TICKS_PER_DEGREE);
        telemetry.addData("Turret Power", power);
    }

    public void setTargetAngle(double degrees) {
        targetTicks = degrees * TICKS_PER_DEGREE;
    }
    public void setTargetTicks(double ticks) {
        targetTicks = ticks;
    }
    public double getCurrentAngle() {
        return motor.getCurrentPosition() / TICKS_PER_DEGREE;
    }
    public double getCurrentTicks() {
        return motor.getCurrentPosition();
    }

    public void stop() {
        motor.setPower(0);
    }

    public void resetEncoder() {
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        targetTicks = 0;
        pid.reset();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}