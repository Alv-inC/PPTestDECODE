package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class flyWheel {
    private final HardwareMap hardwareMap;
    private static PIDController spinPID;
    public DcMotorEx fly1;
    private DcMotorEx fly2;
    public static double p = 0, i = 0, d = 0;
    public static double ff = 0;
    public static double targetVelocity = 0;
    private Telemetry telemetry;

    //ff
    public static double kS = 0, kV = 0, kA = 0;
    private SimpleMotorFeedforward feedforward;
    public flyWheel(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;

        spinPID = new PIDController(p, i, d);
        spinPID.setPID(p,i,d);

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        fly1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "lift1"), 0.005);
        fly2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "lift2"), 0.005);

        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //maybe
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void update() {
            double currentVelocity = fly1.getVelocity();
            double pidOutput = spinPID.calculate(currentVelocity, targetVelocity);
            double ffOutput = feedforward.calculate(targetVelocity);

            double power = pidOutput + ffOutput;

            power = Math.max(-1.0, Math.min(1.0, power));

            // Apply power to motor
            fly1.setPower(power);
            fly2.setPower(power);

            // Optional telemetry for tuning
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        public void setTargetVelocity(double v){
            targetVelocity = v;
        }

        public boolean atSpeed() {
            return Math.abs(fly1.getVelocity() - targetVelocity) < 50; // adjust tolerance
        }
}
