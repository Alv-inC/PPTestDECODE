package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class turret2 {

    private DcMotorEx encoderMotor;   // Encoder attached to small gear
    public CRServo servo1;           // Continuous servo 1
    public CRServo servo2;           // Continuous servo 2

    public double kP = 0.01;         // Proportional constant (tune this)
    private double targetPosition = 0;

    public turret2(HardwareMap hardwareMap, String encoderName, String servo1Name, String servo2Name) {
        encoderMotor = hardwareMap.get(DcMotorEx.class, encoderName);
        servo1 = hardwareMap.get(CRServo.class, servo1Name);
        servo2 = hardwareMap.get(CRServo.class, servo2Name);
    }

    // Set the turret target position (in encoder ticks)
    public void setTargetPosition(double position) {
        this.targetPosition = position;
    }

    // Call this repeatedly in loop() or periodic updates
    public void update() {
        double currentPos = encoderMotor.getCurrentPosition();
        double error = targetPosition - currentPos;

        // Basic proportional control
        double power = error * kP;

        // Clamp power between -1 and 1
        power = Math.max(-1.0, Math.min(1.0, power));

        // Apply same power to both CRServos
        servo1.setPower(power);
        servo2.setPower(power);
    }

    // Optional: get current position
    public double getCurrentPosition() {
        return encoderMotor.getCurrentPosition();
    }
}
