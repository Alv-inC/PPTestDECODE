package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
public class intake {
    private final HardwareMap hardwareMap;
    private CachingDcMotorEx intake;

    public intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"), 0.005);

    }

    public void go(){
        intake.setPower(1);
    }
    public void goSlow(){
        intake.setPower(0.8);
    }


}
