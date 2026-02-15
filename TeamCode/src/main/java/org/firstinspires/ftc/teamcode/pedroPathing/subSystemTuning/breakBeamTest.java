package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BreakBeamTest")
public class breakBeamTest extends OpMode {
    private boolean object1Detected;
    private boolean object2Detected;
    private boolean object3Detected;
    private DigitalChannel beamSensor1;
    private DigitalChannel beamSensor2;
    private DigitalChannel beamSensor3;



    @Override
    public void init() {
        beamSensor1 = hardwareMap.get(DigitalChannel.class, "bb1");
        beamSensor1.setMode(DigitalChannel.Mode.INPUT);
        beamSensor2 = hardwareMap.get(DigitalChannel.class, "bb2");
        beamSensor2.setMode(DigitalChannel.Mode.INPUT);
        beamSensor3 = hardwareMap.get(DigitalChannel.class, "bb3");
        beamSensor3.setMode(DigitalChannel.Mode.INPUT);


    }

    @Override
    public void loop() {
        // Read the sensor: false = broken (object detected), true = not broken
        object1Detected = !beamSensor1.getState();
        object2Detected = !beamSensor2.getState();
        object3Detected = !beamSensor3.getState();

        if (object1Detected & object2Detected & object3Detected) {
            telemetry.addData("Status", "FULLL");
            gamepad1.rumble(1.0,1.0,2000);
        } else {
            telemetry.addData("Status", "NOT FULLL");
        }
    }
}
