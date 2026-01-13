package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testServo")
@Config
public class testServo extends OpMode {

    public static double pos, pos2 = 0; // Adjustable from FTC Dashboard
    CRServo block;
    CRServo servo2;

    @Override
    public void init() {

        block = hardwareMap.get(CRServo.class, "turret1");
        servo2 = hardwareMap.get(CRServo.class, "turret2");
    }

    @Override
    public void loop() {

        block.setPower(pos);
        servo2.setPower(pos2);
    }
}
