package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
//0.76 down
//0.53 up

//camera
//0.5 straight, , 0.6 up
@TeleOp(name = "testServo")
@Configurable
public class testServo extends OpMode {

    public static double ph = 0.58; // Adjustable from FTC Dashboard
    public static double pc = 0.5;
    public static double pb = 0.58;
    Servo block, camera, hood;

    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
        camera = hardwareMap.get(Servo.class, "camera");
        block = hardwareMap.get(Servo.class, "block");
        block.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        hood.setPosition(ph);
        camera.setPosition(pc);
        block.setPosition(pb);

    }
}
