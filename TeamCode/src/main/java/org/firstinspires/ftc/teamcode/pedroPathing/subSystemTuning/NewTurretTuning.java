package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;


import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.TurretPLUSIntake;

@Configurable
@TeleOp(name = "Turret Tuning [NEW]", group = "Tuning")
public class NewTurretTuning extends OpMode {
    private final ElapsedTime timer = new ElapsedTime();
    private TurretPLUSIntake turret;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        turret = new TurretPLUSIntake(hardwareMap, telemetry);
        telemetry.addLine("Turret Tuning Ready!");
        telemetry.addLine(String.valueOf(turret.getCurrentPosition()));
        telemetry.update();



        timer.reset();
        updateSignals();
    }

    @Override
    public void loop() {
        turret.update();

        updateSignals();
    }


    private void updateSignals() {
        double t = timer.seconds();

        panelsTelemetry.addData("position", turret.getCurrentPosition());
        panelsTelemetry.addData("goal position", TurretPLUSIntake.targetPosition);
        panelsTelemetry.addData("Power RN", turret.getPow());
        panelsTelemetry.update();
    }

}