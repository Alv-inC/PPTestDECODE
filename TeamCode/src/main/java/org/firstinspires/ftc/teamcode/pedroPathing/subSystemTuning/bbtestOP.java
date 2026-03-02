package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "BBBBBB")
public class bbtestOP extends OpMode{
    private DcMotorEx intake;
    private boolean object1Detected;
    private boolean object2Detected;
    private boolean object3Detected;
    private DigitalChannel beamSensor1;
    private DigitalChannel beamSensor2;
    private DigitalChannel beamSensor3;

    //finding the port
//    private boolean object4Detected;
//    private boolean object5Detected;
//    private boolean object6Detected;
//    private DigitalChannel beamSensor4;
//    private DigitalChannel beamSensor5;
//    private DigitalChannel beamSensor6;
//
//    private boolean object7Detected;
//    private boolean object8Detected;
//    private DigitalChannel beamSensor7;
//    private DigitalChannel beamSensor8;

    public boolean isFull() {
        object1Detected = !beamSensor1.getState();
        object2Detected = !beamSensor2.getState();
        object3Detected = !beamSensor3.getState();


        if (object1Detected & object2Detected & object3Detected) {
            return true;
        } else {
            return false;
        }
    }

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
        object1Detected = !beamSensor1.getState();
        object2Detected = !beamSensor2.getState();
        object3Detected = !beamSensor3.getState();

        if(object1Detected && object2Detected && object3Detected){
            gamepad1.rumble(1, 1, 500);
        }
        gamepad1.rumble(3000);
        telemetry.addData("BreakBeam 1 Status: ", object1Detected);
        telemetry.addData("BreakBeam 2 Status: ", object2Detected);
        telemetry.addData("BreakBeam 3 Status: ", object3Detected);
    }
//        //finding ports
//        object4Detected = !beamSensor4.getState();
//        object5Detected = !beamSensor5.getState();
//        object6Detected = !beamSensor6.getState();
//        object7Detected = !beamSensor7.getState();
//        object8Detected = !beamSensor8.getState();

//        if (object1Detected){
//            telemetry.addData("Status", "ONE");
//        }
//        if (object2Detected){
//            telemetry.addData("Status", "TWO");
//        }
//        if (object3Detected){
//            telemetry.addData("Status", "THREE");
//        }
//        if (object4Detected){
//            telemetry.addData("Status", "FOUR");
//        }
//        if (object5Detected){
//            telemetry.addData("Status", "FIVE");
//        }
//        if (object6Detected){
//            telemetry.addData("Status", "SIX");
//        }
//        if (object7Detected){
//            telemetry.addData("Status", "SEVEN");
//        }
//        if (object8Detected){
//            telemetry.addData("Status", "EIGHT");
//        }
}
