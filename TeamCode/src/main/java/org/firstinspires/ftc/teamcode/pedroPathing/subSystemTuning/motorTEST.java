package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "areWeCooked")
public class motorTEST extends OpMode {

    private DcMotorEx shooterL;
            private DcMotorEx shooterR;
    @Override
    public void init() {
        shooterL = hardwareMap.get(DcMotorEx.class, "shooterLeft");
        shooterR = hardwareMap.get(DcMotorEx.class, "shooterRight");
        shooterL.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    @Override
    public void loop() {
        if(gamepad1.a){
            shooterL.setPower(1);

        }
        if(gamepad1.b){
            shooterR.setPower(1);
        }
    }
}
