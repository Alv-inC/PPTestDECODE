package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Configurable
public class TurretPLUSIntake {
    // Scan mode
    public static boolean scanEnabled = false;
    public static double scanMinDeg = -90;
    public static double scanMaxDeg = 90;
    public static double searchIncrement = 1;
    public static int searchDirection = 1;
    private double scanTargetDeg = 0;

    private final CRServo leftServo;
    private final CRServo rightServo;
    private final DcMotorEx encoder;

    private final PIDController pid;
    private final Telemetry telemetry;

    // PID
    public static double p = 0.00013;
    public static double i = 0.0;
    public static double d = 0.000;
    public static double currentTicks;

    // Motion
    public double targetPosition = 0;

    public static double power;

    // Gear math
    public static double SMALL_GEAR_TICKS_PER_REV = 8192;
    public static double GEAR_RATIO = (double) 125 /33; // big / small
    public static double TICKS_PER_TURRET_REV =
            SMALL_GEAR_TICKS_PER_REV * GEAR_RATIO;
    public static double TICKS_PER_DEGREE =
            TICKS_PER_TURRET_REV / 360.0;

    public TurretPLUSIntake(HardwareMap hardwareMap, Telemetry telemetry, DcMotorEx encoder1) {
        this.telemetry = telemetry;

        leftServo = hardwareMap.get(CRServo.class, "turret2");
        rightServo = hardwareMap.get(CRServo.class, "turret1");
        this.encoder = encoder1;
        resetEncoder();
        pid = new PIDController(p, i, d);
        pid.setPID(p, i, d);
    }

    public void update() {
        pid.setPID(p, i, d);
        if (scanEnabled) {
            scanTargetDeg += searchIncrement * searchDirection;
            setTargetAngle(scanTargetDeg);
            if (scanTargetDeg >= scanMaxDeg) {
                scanTargetDeg = scanMaxDeg;
                searchDirection = -1;
            } else if (scanTargetDeg <= scanMinDeg) {
                scanTargetDeg = scanMinDeg;
                searchDirection = 1;
            }
        }

        currentTicks = encoder.getCurrentPosition();
        power = pid.calculate(currentTicks, targetPosition);
        power = clamp(power, -1.0, 1.0);
        leftServo.setPower(power);
        rightServo.setPower(power);

        telemetry.addData("Turret Pos", currentTicks);
        telemetry.addData("Turret Target", targetPosition);
        telemetry.addData("Turret Power", power);
    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public double getCurrentPosition() {
        return encoder.getCurrentPosition();
    }
//    public void setTargetAngle(double degrees) {
//        targetPosition = degrees * TICKS_PER_DEGREE;
//    }

    public double getCurrentAngle() {
        return getCurrentPosition() / TICKS_PER_DEGREE;
    }
    public void setTargetAngle(double degrees) {
        targetPosition = degrees * TICKS_PER_DEGREE;
    }
    public double getPow(){
        return power;
    }
    public boolean isClose(double toleranceTicks) {
        return Math.abs(getCurrentPosition() - targetPosition) < toleranceTicks;
    }

    public void stop() {
        leftServo.setPower(0);
        rightServo.setPower(0);
    }

    public void resetEncoder() {
        encoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        targetPosition = 0;
    }
    public void startScan() {
        scanEnabled = true;
        scanTargetDeg = clamp(getCurrentAngle(), scanMinDeg, scanMaxDeg);
    }

    public void stopScan() {
        scanEnabled = false;
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

//    public void intakeStop(){
//        encoder.setPower(0);
//    }
//    public void intakeMed(){
//        encoder.setPower(0.5);
//    }
//    public void intakeFull(){
//        encoder.setPower(1);
//    }
//    public void intakeBack(){
//        encoder.setPower(-1);
//    }


}
