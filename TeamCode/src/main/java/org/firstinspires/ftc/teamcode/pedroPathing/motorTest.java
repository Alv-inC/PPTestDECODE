package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorControl")
@Config
public class motorTest extends OpMode {

    private DcMotorEx fly1;
    @Override
    public void init() {
        fly1 = hardwareMap.get(DcMotorEx.class, "motor");

    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            fly1.setPower(0.3);

        }
    }
}
