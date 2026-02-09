package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//0.76 down
//0.53 up

//camera
//0.12 straight, 0.03 up, 0.19 down
@TeleOp(name = "testServo")
@Config
public class testServo extends OpMode {

    public static double pc, pb, pl, pr = 0; // Adjustable from FTC Dashboard
    Servo block, camera, hl, hr;

    @Override
    public void init() {
        camera = hardwareMap.get(Servo.class, "camera");
        block = hardwareMap.get(Servo.class, "block");
        hl = hardwareMap.get(Servo.class, "hoodLeft");
        hr = hardwareMap.get(Servo.class, "hoodRight");
    }

    @Override
    public void loop() {
        camera.setPosition(pc);
        block.setPosition(pb);
        hl.setPosition(pl);
        hr.setPosition(pr);
    }
}
