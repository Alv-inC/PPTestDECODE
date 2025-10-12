package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
public class Turret {
    private static PIDController turretPID;
    CachingDcMotorEx turretMotor;

    public static double p = 0, i = 0, d = 0;
    public static double targetPosition = 0;
    //what is for what
    private Telemetry telemetry;
    public static double divide;


    public Turret(HardwareMap hardwareMap) {
        turretPID = new PIDController(p, i, d);
        turretPID.setPID(p, i, d);
        turretMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"), 0.005);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    //
    public void update() {
        turretPID.setPID(p, i, d);
        int pos = turretMotor.getCurrentPosition();

        double power = (turretPID.calculate(pos, targetPosition));
        turretMotor.setPower(power);

    }

    public void setTargetPosition(double targetPosition) {
        this.targetPosition = targetPosition;
    }

    public int getCurrentPosition() {
        return turretMotor.getCurrentPosition();
    }

    public boolean isClose() {
        return Math.abs(turretMotor.getCurrentPosition() - targetPosition) < 45;
    }

    public void setPower(double power) {
        this.setPower(power);
    }
}
