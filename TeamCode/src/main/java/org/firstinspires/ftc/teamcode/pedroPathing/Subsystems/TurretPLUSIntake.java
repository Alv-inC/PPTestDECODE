package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Configurable
public class TurretPLUSIntake {
    // ===== Limits =====
    public static double MAX_ANGLE_DEG = 100.0; // +/- 100 degrees from zero (no offset)

    // ===== Scan mode (optional) =====
    public static boolean scanEnabled = false;
    public static double scanMinDeg = -90;
    public static double scanMaxDeg = 90;
    public static double searchIncrement = 1;
    public static int searchDirection = 1;
    private double scanTargetDeg = 0;

    // ===== Hardware =====
    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx encoder;

    private final PIDController pid;
    private final Telemetry telemetry;

    // ===== PID =====
    public static double p = 0.0006;
    public static double i = 0.0;
    public static double d = 0.00007;

    public static double currentTicks;
    public static double power;

    // ===== Motion =====
    public double targetPosition = 0;

    // Hold (simple deadband)
    public static double HOLD_TOLERANCE_TICKS = 300; // tune this

    // ===== Gear math =====
    public static double SMALL_GEAR_TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = (double) 125 / 33; // big / small
    public static double TICKS_PER_TURRET_REV = SMALL_GEAR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEGREE = TICKS_PER_TURRET_REV / 360.0;

    public TurretPLUSIntake(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx encoder1) {
        this.telemetry = telemetry;

        leftServo = hardwareMap.get(CRServo.class, "turret2");
        rightServo = hardwareMap.get(CRServo.class, "turret1");
        this.encoder = encoder1;

        resetEncoder();

        pid = new PIDController(p, i, d);
        pid.setPID(p, i, d);
    }

    private double maxTicks() {
        return MAX_ANGLE_DEG * TICKS_PER_DEGREE;
    }

    public void update() {
        pid.setPID(p, i, d);

        // Optional scan logic (kept, but clamped to limits)
        // if (scanEnabled) {
        //     scanTargetDeg += searchIncrement * searchDirection;
        //     setTargetAngle(scanTargetDeg);
        //     if (scanTargetDeg >= scanMaxDeg) { scanTargetDeg = scanMaxDeg; searchDirection = -1; }
        //     else if (scanTargetDeg <= scanMinDeg) { scanTargetDeg = scanMinDeg; searchDirection = 1; }
        // }

        currentTicks = encoder.getCurrentPosition();

        // Clamp target every loop (safety)
        double minTicks = -maxTicks();
        double maxTicks =  maxTicks();
        targetPosition = clamp(targetPosition, minTicks, maxTicks);

        // PID output
        power = pid.calculate(currentTicks, targetPosition);

        // Hold deadband
        if (Math.abs(targetPosition - currentTicks) <= HOLD_TOLERANCE_TICKS) {
            power = 0;
        }

        // Hard-stop protection: never drive farther past the limits
        if (currentTicks <= minTicks && power < 0) power = 0;
        if (currentTicks >= maxTicks && power > 0) power = 0;

        power = clamp(power, -1.0, 1.0);

        leftServo.setPower(power);
        rightServo.setPower(power);

        telemetry.addData("Turret Pos (ticks)", currentTicks);
        telemetry.addData("Turret Target (ticks)", targetPosition);
        telemetry.addData("Turret Pos (deg)", getCurrentAngle());
        telemetry.addData("Turret Target (deg)", targetPosition / TICKS_PER_DEGREE);
        telemetry.addData("Turret Power", power);
        telemetry.addData("Limit Min (deg)", -MAX_ANGLE_DEG);
        telemetry.addData("Limit Max (deg)", MAX_ANGLE_DEG);
    }

    // ===== Target setters (clamped) =====
    public void setTargetPosition(double targetPositionTicks) {
        double minTicks = -maxTicks();
        double maxTicks =  maxTicks();
        this.targetPosition = clamp(targetPositionTicks, minTicks, maxTicks);
    }

    public void setTargetAngle(double degrees) {
        double degClamped = clamp(degrees, -MAX_ANGLE_DEG, MAX_ANGLE_DEG);
        this.targetPosition = degClamped * TICKS_PER_DEGREE;
    }

    // ===== Getters =====
    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }

    public double getCurrentAngle() {
        return getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public double getPow() {
        return power;
    }

    public boolean isClose(double toleranceTicks) {
        return Math.abs(getCurrentPosition() - targetPosition) < toleranceTicks;
    }

    // ===== Helpers =====
    public void stop() {
        leftServo.setPower(0);
        rightServo.setPower(0);
        power = 0;
    }

    public void resetEncoder() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

}