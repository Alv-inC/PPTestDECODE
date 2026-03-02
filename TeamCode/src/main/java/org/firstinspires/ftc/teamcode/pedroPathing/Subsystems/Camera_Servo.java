package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

public class Camera_Servo {
    private Servo hood;
    private final HardwareMap hardwareMap;

    private final double hood_high = 0.63-0.29; //47.5 degrees
    private final double hood_mid = 0.23;
    private final double hood_low = 0.41-0.29; //35.3
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
