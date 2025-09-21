package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

//rb = 2, lb = 3, rf = 1, lf = 0

@TeleOp(name = "TESTTEMP")
public class testTEMP extends OpMode {
    DcMotorEx rf;
    DcMotorEx rb;
    DcMotorEx lb;
    DcMotorEx lf;
    @Override
    public void init() {
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        rb = hardwareMap.get(DcMotorEx.class, "rb");
        lb = hardwareMap.get(DcMotorEx.class, "lb");

    }

    @Override
    public void loop() {
if(gamepad1.a){
    rb.setPower(1);
}
if(gamepad1.b){
    lb.setPower(1);
}
if(gamepad1.x){
    rf.setPower(1);
}
if(gamepad1.y){
    rb.setPower(1);
}
    }
}
