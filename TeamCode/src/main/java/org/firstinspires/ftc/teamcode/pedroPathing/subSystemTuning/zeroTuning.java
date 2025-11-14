package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;

@Configurable
@TeleOp(name ="ZERO")
public class zeroTuning extends OpMode {
    private CRServo block1;
    private CRServo block2;
    private DcMotorEx intake;
    private Turret turret;



    public static double block_val;
    public static double intake_val;



    @Override
    public void init() {
    block1 = hardwareMap.get(CRServo.class, "block1");
    block2 = hardwareMap.get(CRServo.class, "block2");
        block2.setDirection(CRServo.Direction.REVERSE);
    intake = hardwareMap.get(DcMotorEx.class, "intake");
    intake.setDirection(DcMotorSimple.Direction.REVERSE);
        turret = new Turret(hardwareMap, telemetry);




    }

    @Override
    public void loop() {
        if(gamepad1.y){
            block1.setPower(1);
            block2.setPower(1);
        }
        if(gamepad1.a){
            intake.setPower(0.7);
        }
        if(gamepad1.b){
            intake.setPower(1);
        }
        if(gamepad1.x){
            block1.setPower(-1);
            block2.setPower(-1);
        }

    }

}
