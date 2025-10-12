package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;



import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Turret;


@Configurable
@TeleOp(name = "Turret Tuning", group = "Tuning")
public class turretTuning extends OpMode{

    private Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        telemetry.addLine("Turret Tuning Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        turret.update();
    }

}
