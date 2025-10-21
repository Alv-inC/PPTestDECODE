package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name ="ZERO")
public class zeroTuning extends OpMode {
    private Servo hoodLeft;
    private Servo hoodRight;


    @Override
    public void init() {
    hoodLeft = hardwareMap.get(Servo.class, "hoodLeft");
    hoodRight = hardwareMap.get(Servo.class, "hoodRight");


    hoodLeft.setDirection(Servo.Direction.REVERSE);
    hoodRight.setPosition(0);
    hoodLeft.setPosition(0);
    }

    @Override
    public void loop() {
    }

}
