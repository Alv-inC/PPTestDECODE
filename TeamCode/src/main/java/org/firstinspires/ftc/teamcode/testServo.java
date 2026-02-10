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
//0.43 straight, 0.31 down, 0.55 up
@TeleOp(name = "testServo")
@Config
public class testServo extends OpMode {

    public static double pb, pl, pr = 0.53; // Adjustable from FTC Dashboard
    public static double pc = 0.7;
    Servo block, camera;

    @Override
    public void init() {
        camera = hardwareMap.get(Servo.class, "camera");
        block = hardwareMap.get(Servo.class, "block");
    }

    @Override
    public void loop() {
        camera.setPosition(pc);
        //block.setPosition(pb);

    }
}
