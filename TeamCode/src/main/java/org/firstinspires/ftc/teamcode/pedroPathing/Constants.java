package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
    .mass(11.29)
            .forwardZeroPowerAcceleration(-38.585)
            .lateralZeroPowerAcceleration(-64.89)
            .centripetalScaling(0.00015)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0042,0,0.000003,0.6,0.003))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.01, 0, 0.01))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.14, 0, 0.015, 0.1))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.06, 0.02));
    //intakeAngle -> angle of intake
    //intake -> active intake (pinpoint???)
    //bottom lift -> liftB
    //top lift -> liftT

    //m
    public static MecanumConstants driveConstants = new MecanumConstants()
            //adjust later
            .maxPower(1)
            //NEEDS ADJUSTMENT
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .xVelocity(84.628)
            .yVelocity(66.5)
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-5)
            .strafePodX(-7)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 2, 2);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();

    }
}
