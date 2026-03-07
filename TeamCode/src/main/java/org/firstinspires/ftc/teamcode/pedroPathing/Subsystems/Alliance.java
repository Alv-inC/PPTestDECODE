package org.firstinspires.ftc.teamcode.pedroPathing.Subsystems;

import static android.text.AndroidCharacter.mirror;

import com.pedropathing.geometry.Pose;

public enum Alliance {
    RED(new Pose(138, 144)),
    BLUE((new Pose(138, 144)).mirror());

    public static Alliance current = Alliance.RED;
    public final Pose goal;

    Alliance(Pose goal) {
        this.goal = goal;
    }
}