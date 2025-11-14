package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "SingleMotorControl")
@Config
public class motorTest extends OpMode {

    private DcMotor motor;
    public static double motorPower = 0.5; // Adjustable from FTC Dashboard

    @Override
    public void init() {
        motor = hardwareMap.get(DcMotor.class, "intake");
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            motor.setPower(Math.abs(motorPower));
        } else if (gamepad1.y) {
            motor.setPower(-Math.abs(motorPower));
        } else if (gamepad1.b) {
            motor.setPower(0);
        }

        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }
}
