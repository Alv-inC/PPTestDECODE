//package org.firstinspires.ftc.teamcode.math;
//
//import com.acmerobotics.dashboard.config.Config;
//import com.pedropathing.geometry.Pose;
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.Alliance;
//import org.firstinspires.ftc.teamcode.pedroPathing.Subsystems.turretturret;
//
//@Config
//public class TractorBeam {
//    public static double minimumAngleDegrees = -90;
//    private static turretturret turret;
//
//    public static void aimTurret(Pose currentPose, Alliance alliance) {
//        double turretTargetDegrees = getTurretTargetDegrees(currentPose, alliance);
//
//        turret.setTargetDegrees(turretTargetDegrees);
//    }
//
//    public static double getTurretTargetDegrees(Pose currentPose, Telemetry telemetry, Alliance alliance) {
//        double targetAngleDegrees = WaveLength.getAngleWithInterpolation(currentPose, alliance);
//
//        double turretTargetDegrees = AngleUnit.normalizeDegrees(targetAngleDegrees - Math.toDegrees(currentPose.getHeading()));
//
////        double targetAngleRadians = Math.atan2(alliance.goal.getY() - currentPose.getY(), alliance.goal.getX() - currentPose.getX());
////
////        double turretTargetRadians = AngleUnit.normalizeRadians(targetAngleRadians - currentPose.getHeading());
////
////        double turretTargetDegrees = Math.toDegrees(turretTargetRadians);
//
//
//        if (turretTargetDegrees < minimumAngleDegrees) {
//            turretTargetDegrees += 360;
//        }
//
//        telemetry.addData("Auto Aim Target", targetAngleDegrees);
////        telemetry.addData("Auto Aim Target", Math.toDegrees(targetAngleRadians));
//        telemetry.addData("Auto Aim Turret Target", turretTargetDegrees);
//
//
//        return turretTargetDegrees;
//    }
//}