package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "areWeCooked")
public class motorTEST extends OpMode {

            private Servo test;
            public static double testVal;
    @Override
    public void init() {
        test = hardwareMap.get(Servo.class, "testMotor");

    }

    @Override
    public void loop() {
        test.setPosition(testVal);


    }
}
