package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

import dev.frozenmilk.dairy.cachinghardware.CachingDcMotorEx;

@TeleOp(name = "workkk")
public class idkatthispoint extends OpMode {
    private DcMotorEx fly1;
            private DcMotorEx fly2;
            private DcMotorEx turretMotor;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();


    @Override
    public void init() {
        fly1 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterRight"), 0.005);
        fly2 = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "shooterLeft"), 0.005);



        fly1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fly2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        fly1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        fly2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //maybe
        fly1.setDirection(DcMotorSimple.Direction.REVERSE);


        turretMotor = new CachingDcMotorEx(hardwareMap.get(DcMotorEx.class, "turret"), 0.005);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        panelsTelemetry.addData("flyR", fly1.getVelocity());
        panelsTelemetry.addData("flyL", fly2.getVelocity());
        panelsTelemetry.addData("turret", turretMotor.getCurrentPosition());
        panelsTelemetry.update();

        if(gamepad1.a){
            fly1.setPower(1);
        }
        if(gamepad1.b){
            fly2.setPower(1);
        }
        if(gamepad1.x){
            turretMotor.setPower(0.1);
        }

    }
}
