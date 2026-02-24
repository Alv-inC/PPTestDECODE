    package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@Configurable
public class intake {
    private final HardwareMap hardwareMap;
    private CachingDcMotorEx intake;

    public intake(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        intake = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "intake"), 0.005);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void go(){
        intake.setPower(0.95);
    }
    public void goSlowFirst(){
        intake.setPower(0.62);
    }
    public void goSlow(){
        intake.setPower(0.4);
    }

    public double getCurrent(){
        return intake.getCurrent(CurrentUnit.AMPS);
    }


}
