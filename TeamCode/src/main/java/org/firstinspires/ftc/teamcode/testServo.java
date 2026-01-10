package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testServo")
@Config
public class testServo extends OpMode {

    public static double pos = 0; // Adjustable from FTC Dashboard
    Servo block;

    @Override
    public void init() {
        block = hardwareMap.get(Servo.class, "camera");
    }

    @Override
    public void loop() {
        block.setPosition(pos);
    }
}
