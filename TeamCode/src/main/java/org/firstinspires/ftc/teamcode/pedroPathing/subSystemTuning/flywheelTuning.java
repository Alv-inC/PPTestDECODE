package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class flywheelTuning extends OpMode {

    private flyWheel flywheel;
    private double velocityStep = 100; // RPM or ticks/sec
    public static double target;

    @Override
    public void init() {
        flywheel = new flyWheel(hardwareMap);
        telemetry.addLine("Flywheel Tuning Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        flywheel.update();
    }
}

