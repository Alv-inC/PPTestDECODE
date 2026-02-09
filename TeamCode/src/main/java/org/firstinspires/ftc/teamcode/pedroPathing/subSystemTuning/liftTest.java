package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "LIFTTTT")
public class liftTest extends OpMode {
    private DcMotorEx lift;
    private DigitalChannel bb;

    private boolean bb_state;
    @Override
    public void init() {
        bb= hardwareMap.get(DigitalChannel.class, "bb");
        lift = hardwareMap.get(DcMotorEx.class, "lift");

        bb.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {
        bb_state = bb.getState();
        if(!bb_state){
            lift.setPower(-1);
        }
        if(gamepad1.b){
            lift.setPower(1);
        }

    }
}
