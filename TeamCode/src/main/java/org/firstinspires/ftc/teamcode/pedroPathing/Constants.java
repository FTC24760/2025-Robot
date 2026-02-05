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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3)
            .forwardZeroPowerAcceleration(-31)
            .lateralZeroPowerAcceleration(-60)
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.06, 0.03, 0.000005, 0.6, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.1, 0.01));

    //public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 4.0, 4.0);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pinpointLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frDrive")
            .rightRearMotorName("rrDrive")
            .leftRearMotorName("rlDrive")
            .leftFrontMotorName("flDrive")
            .leftFrontMotorDirection(DcMotor.Direction.REVERSE)
            .leftRearMotorDirection(DcMotor.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotor.Direction.FORWARD)
            .rightRearMotorDirection(DcMotor.Direction.FORWARD)
            .xVelocity(74)
            .yVelocity(54)
    ;
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.5) // vertical distance from horizontal center of robot
            .strafePodX(-4.5) // horizontal distance from vertical center of robot
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);
}
