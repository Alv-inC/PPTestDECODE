package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;

@Configurable
@TeleOp(name ="ZERO")
public class zeroTuning extends OpMode {
    private Servo a;
    private Servo b;
    private Servo c;




    public static double val1;
    public static double val2;
    private DcMotorEx intake;


    @Override
    public void init() {
    a = hardwareMap.get(Servo.class, "hood");
    b = hardwareMap.get(Servo.class, "block");
    b.setDirection(Servo.Direction.REVERSE);
    c = hardwareMap.get(Servo.class, "camera");
    intake = hardwareMap.get(DcMotorEx.class, "intake");

//        ServoControllerEx controller = (ServoControllerEx) a.getController();
//        int port = a.getPortNumber();
//        controller.setServoPwmRange(port, new PwmControl.PwmRange(500, 2500));


    }

    @Override
    public void loop() {
        a.setPosition(val1);
        b.setPosition(val2);
        c.setPosition(0);
        intake.setPower(-1);


    }

}
