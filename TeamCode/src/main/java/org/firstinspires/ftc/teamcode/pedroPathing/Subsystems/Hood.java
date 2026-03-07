package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
@Configurable
public class Hood {
    private Servo hood;
    private final HardwareMap hardwareMap;

    private static double hood_high_far = 0.16; //47.5 degrees
    private static double hood_high = 0.38; //47.5 degrees
    private final double hood_mid = 0.35;
    private final double hood_low = 0.54; //35.3
    private final double auto = 0.21;
    public Hood(HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        hood = hardwareMap.get(Servo.class, "hood");
        hood.setDirection(Servo.Direction.REVERSE);
    }


    public void setLow(){
        hood.setPosition(hood_low);
    }
    public void setMid(){
        hood.setPosition(hood_mid);
    }
    public void setHigh() {hood.setPosition(hood_high);}
    public void setHighFar(){
        hood.setPosition(hood_high_far);
    }
    public void setAuto() {hood.setPosition(auto);}

}
