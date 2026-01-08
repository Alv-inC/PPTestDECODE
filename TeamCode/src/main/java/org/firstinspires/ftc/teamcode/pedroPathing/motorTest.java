package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "DualMotorControl")
@Config
public class motorTest extends OpMode {

    private DcMotor fly1, fly2, intake;
    public static double motorPower = 0.5; // Adjustable from FTC Dashboard

    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotor.class, "fly1");
        fly2 = hardwareMap.get(DcMotor.class, "fly2");
        intake = hardwareMap.get(DcMotor.class, "intake");
        telemetry = FtcDashboard.getInstance().getTelemetry();
        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            fly1.setPower(Math.abs(motorPower));
            fly2.setPower(Math.abs(motorPower));
            intake.setPower(Math.abs(motorPower));


        } else if (gamepad1.y) {
            fly2.setPower(-Math.abs(motorPower));
            fly2.setPower(Math.abs(motorPower));
            intake.setPower(-Math.abs(motorPower));
        } else if (gamepad1.b) {
            fly1.setPower(0);
            fly2.setPower(0);
            intake.setPower(0);
        }

        telemetry.update();
    }
}
