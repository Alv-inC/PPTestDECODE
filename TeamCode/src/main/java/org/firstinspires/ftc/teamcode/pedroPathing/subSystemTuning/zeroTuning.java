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




    public static double block_val;
    public static double intake_val;



    @Override
    public void init() {
    a = hardwareMap.get(Servo.class, "");
    b = hardwareMap.get(Servo.class, "block2");
    c = hardwareMap.get(Servo.class, "intake");


        ServoControllerEx controller = (ServoControllerEx) a.getController();
        int port = a.getPortNumber();
        controller.setServoPwmRange(port, new PwmControl.PwmRange(500, 2500));


    }

    @Override
    public void loop() {


    }

}
