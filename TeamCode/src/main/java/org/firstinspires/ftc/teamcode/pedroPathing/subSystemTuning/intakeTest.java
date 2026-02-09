package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@Configurable
@TeleOp(name = "intakeTESTTTT")
public class intakeTest extends OpMode {
    public static double intakePOW;
    DcMotorEx intake;
    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
    }

    @Override
    public void loop() {
        if(gamepad1.a){
            intake.setPower(1);
        }
        if(gamepad1.x){
            intake.setPower(0.8);
        }
        if(gamepad1.b){
            intake.setPower(-1);
        }
        if(gamepad1.y){
            intake.setPower(-0.8);
        }
    }
}
