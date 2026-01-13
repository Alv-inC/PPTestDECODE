package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.Servo;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class flyWheel {
    private final HardwareMap hardwareMap;
    private static PIDController spinPID;
    public CachingDcMotorEx fly1;
    private CachingDcMotorEx fly2;

    private Servo block;


    public static double p = 0.15, i = 0, d = 0;
    public static double ff = 0;
    public static double targetVelocity = 0;
    public static double power = 0;
    public static double currentVelocity = 0;
    private Telemetry telemetry;

    //ff
    public static double kS = 0, kV = 0, kA = 0;
    private SimpleMotorFeedforward feedforward;
    public flyWheel(HardwareMap hardwareMap, Telemetry t) {
        this.hardwareMap = hardwareMap;
        this.telemetry = t;

        spinPID = new PIDController(p, i, d);
        spinPID.setPID(p,i,d);

        feedforward = new SimpleMotorFeedforward(kS, kV, kA);

        fly1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterRight"), 0.005);
        fly2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterLeft"), 0.005);

        //blocker
        block = hardwareMap.get(Servo.class, "block");
        //block.setDirection(Servo.Direction.REVERSE);
        block.setPosition(0.6);

        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //maybe
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void update() {
            spinPID.setPID(p,i,d);
             currentVelocity = fly1.getVelocity();
            double pidOutput = spinPID.calculate(currentVelocity, targetVelocity);

            //make sure its correct units
            double ffOutput = feedforward.calculate(targetVelocity);

             power = pidOutput + ffOutput;

            //power = Math.max(-1.0, Math.min(1.0, power));

            // Apply power to motor
            fly1.setPower(power);
            fly2.setPower(power);

           //  Optional telemetry for tuning
            telemetry.addData("Target Velocity", targetVelocity);
            telemetry.addData("Current Velocity", currentVelocity);
            telemetry.addData("Power", power);
            telemetry.update();
        }

        public void setTargetVelocity(double v){
            targetVelocity = v;
        }

 //
    public void constantShootAuto(){
        p = 0.15;
        targetVelocity = -1300;
    }
        public void constantShoot(){
            p = 0.15;
           targetVelocity = -1200;
        }
        public void uppies(){
            block.setPosition(0);
        }
    public void downies(){
        block.setPosition(0.6);
    }
    public void zero(){
        block.setPosition(0);
    }


    public void constantShootSlow(){
        p = 0.15;
        targetVelocity = -1000;
    }
    public void constantShootAtVelocity(int v){
        p = 0.15;
        targetVelocity = v;
        new WaitCommand(2500);
    }

    public void constantStop(){
        block.setPosition(0.6);
        //p = 0;
        //targetVelocity = 0;
    }
    public boolean isCurrentVelocityEnough(){
        return fly1.getVelocity() <= -1300;
    }

        public boolean atSpeed() {
            return Math.abs(fly1.getVelocity() - targetVelocity) < 50; // adjust tolerance
        }
}
