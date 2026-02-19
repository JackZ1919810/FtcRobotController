package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {


    //PIDF constant setup: zero power accelerations, PIDF coefficients
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.1)
            .forwardZeroPowerAcceleration(-60.861288517480833)
            .lateralZeroPowerAcceleration(-45.842144532334667)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.052, 0, 0.001, 0.021))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.006, 0.007))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0, 0.0001, 0.6, 0.021))
            .centripetalScaling(0.0006)
            ;

    //Motor constant setup: configuration, direction, velocity
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRwheel")
            .rightRearMotorName("BRwheel")
            .leftRearMotorName("BLwheel")
            .leftFrontMotorName("FLwheel")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(44.930140709190567)
            .yVelocity(52.0244335373202);

    //Encoder localizations
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()

            .forwardTicksToInches(0.001990565001921)
            .strafeTicksToInches(0.001963543916)
            .turnTicksToInches(0.002033369892898)
            .leftPodY(-112/25.4)
            .rightPodY(112/25.4)
            .strafePodX(126/25.4)
            .leftEncoder_HardwareMapName("FRwheel")
            .rightEncoder_HardwareMapName("BRwheel")
            .strafeEncoder_HardwareMapName("FLwheel")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP));

    //Path constraints (behaviour setup, eg, when to stop in a path)
    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.91,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}

