package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Camera_Servo {
    private Servo hood;
    private final HardwareMap hardwareMap;

    private final double hood_high = 0.42; //
    private final double hood_mid = 0.38;
    private final double hood_low = 0.41-0.29+0.15; //
    public Camera_Servo(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        hood = hardwareMap.get(Servo.class, "camera");
    }


    public void setLow(){
        hood.setPosition(hood_low);
    }
    public void setMid(){
        hood.setPosition(hood_mid);
    }
    public void setHigh(){
        hood.setPosition(hood_high);
    }

}
