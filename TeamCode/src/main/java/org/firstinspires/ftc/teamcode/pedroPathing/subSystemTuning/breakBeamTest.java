package org.firstinspires.ftc.teamcode.pedroPathing.subSystemTuning;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class breakBeamTest {
    private DigitalChannel beamSensor1;
    private DigitalChannel beamSensor2;
    private DigitalChannel beamSensor3;

    private boolean object1Detected, object2Detected, object3Detected;

    // For edge detect
    private boolean prev1 = false, prev2 = false, prev3 = false;

    // Latched hit flag (set true once, then cleared when read)
    private boolean hitAnyLatched = false;

    // Debounce / cooldown so one ball doesn't spam hits
    private static final long HIT_COOLDOWN_MS = 200;
    private long lastHitMs = 0;

    public breakBeamTest(HardwareMap hardwareMap){
        beamSensor1 = hardwareMap.get(DigitalChannel.class, "bb1");
        beamSensor1.setMode(DigitalChannel.Mode.INPUT);

        beamSensor2 = hardwareMap.get(DigitalChannel.class, "bb2");
        beamSensor2.setMode(DigitalChannel.Mode.INPUT);

        beamSensor3 = hardwareMap.get(DigitalChannel.class, "bb3");
        beamSensor3.setMode(DigitalChannel.Mode.INPUT);
    }

    /** Call once per OpMode loop. */
    public void update() {
        boolean cur1 = !beamSensor1.getState(); // beam broken
        boolean cur2 = !beamSensor2.getState();
        boolean cur3 = !beamSensor3.getState();

        long now = System.currentTimeMillis();

        boolean hit1 = cur1 && !prev1;
        boolean hit2 = cur2 && !prev2;
        boolean hit3 = cur3 && !prev3;

        if ((hit1 || hit2 || hit3) && (now - lastHitMs) >= HIT_COOLDOWN_MS) {
            hitAnyLatched = true;
            lastHitMs = now;
        }

        prev1 = cur1; prev2 = cur2; prev3 = cur3;

        object1Detected = cur1;
        object2Detected = cur2;
        object3Detected = cur3;
    }

    /** Returns true once per hit, then resets. */
    public boolean consumeHitAny() {
        boolean out = hitAnyLatched;
        hitAnyLatched = false;
        return out;
    }

    public boolean isFull() {
        return object1Detected && object2Detected && object3Detected;
    }
}