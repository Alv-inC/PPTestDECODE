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

            private DcMotorEx rf;
    private DcMotorEx rb;
    private DcMotorEx lf;
    private DcMotorEx lb;
    @Override
    public void init() {
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lb = hardwareMap.get(DcMotorEx.class, "lb");

    }

    @Override
    public void loop() {

        if(gamepad1.a){
            rf.setPower(1);
        }
        if(gamepad1.b){
            rb.setPower(1);
        }
        if(gamepad1.x){
            lf.setPower(1);
        }
        if(gamepad1.y){
            lb.setPower(1);
        }

    }
}
