package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "LIFTTTT")
public class liftTest extends OpMode {
    private DcMotorEx lift;
    @Override
    public void init() {
        lift = hardwareMap.get(DcMotorEx.class, "lift");
    }

    @Override
    public void loop() {
        lift.setPower(-1);

    }
}
