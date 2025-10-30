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

        intake.setPower(intakePOW);
    }
}
