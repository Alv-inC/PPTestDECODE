package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;



import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class turretTuning extends OpMode{
    private final ElapsedTime timer = new ElapsedTime();
    private Turret turret;
    private final TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    @Override
    public void init() {
        turret = new Turret(hardwareMap, telemetry);
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
        panelsTelemetry.addData("goal position", Turret.targetPosition);
        panelsTelemetry.update();
    }

}
