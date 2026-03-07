package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class BreakPad {
    private Servo left, right;
    private final HardwareMap hardwareMap;
    private final double ldown = 0.18;
    private final double lup = 0.001; //
    private final double rdown = 0.33;
    private final double rup = 0.37; //
    public BreakPad(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        left = hardwareMap.get(Servo.class, "brakeLeft");
        right = hardwareMap.get(Servo.class, "brakeRight");
        left.setDirection(Servo.Direction.REVERSE);
//        right.setDirection(Servo.Direction.REVERSE);
    }


    public void setDown(){
        left.setPosition(ldown);
        right.setPosition(rdown);
    }
    public void setUp(){
        left.setPosition(lup);
        right.setPosition(rup);
    }

}
