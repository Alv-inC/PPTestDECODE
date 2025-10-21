package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.flyWheel;

@Configurable
@TeleOp(name = "Flywheel Tuning", group = "Tuning")
public class flywheelTuning extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();
    private flyWheel flywheel;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    private double velocityStep = 100; // RPM or ticks/sec
    public static double target;

    @Override
    public void init() {
        flywheel = new flyWheel(hardwareMap, telemetry);
//        telemetry.addLine("Flywheel Tuning Ready!");
//        telemetry.update();

    }

    @Override
    public void loop() {
        flywheel.update();
        updateSignals();
    }

    private void updateSignals() {
        double t = timer.seconds();


        panelsTelemetry.addData("current speed", flyWheel.currentVelocity);
        panelsTelemetry.addData("goal speed", flyWheel.targetVelocity);
        panelsTelemetry.update();
    }
}

