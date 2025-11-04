package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name ="ZERO")
public class zeroTuning extends OpMode {
    private CRServo block;

    private Servo hood;

    public static double hood_val;
    public static double block_val;



    @Override
    public void init() {
    hood = hardwareMap.get(Servo.class, "hood");
    block = hardwareMap.get(CRServo.class, "block");
    hood.setDirection(Servo.Direction.REVERSE);




    }

    @Override
    public void loop() {
        hood.setPosition(hood_val);
        block.setPower(block_val);
    }

}
