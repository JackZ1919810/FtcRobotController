package org.firstinspires.ftc.teamcode.pedroPathing;

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
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.7)
            .forwardZeroPowerAcceleration(-62.63123937567751)
            .lateralZeroPowerAcceleration(-57.1686149891093)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.052, 0, 0.001, 0.021))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.006, 0.007))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.6, 0, 0.0001, 0.6, 0.021))
            .centripetalScaling(0.0006)
            ;


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("FRwheel")
            .rightRearMotorName("BRwheel")
            .leftRearMotorName("BLwheel")
            .leftFrontMotorName("FLwheel")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(61.76695591880741)
            .yVelocity(61.573318208052683);


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            /*.forwardTicksToInches(-0.01434737309)
            .strafeTicksToInches(-0.002189522345)
            .turnTicksToInches(.001989436789)
            .leftPodY(112/25.4)
            .rightPodY(-112/25.4)
            .strafePodX(-112/25.4)
            .leftEncoder_HardwareMapName("BLwheel")
            .rightEncoder_HardwareMapName("BRwheel")
            .strafeEncoder_HardwareMapName("shooter_right")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
    */

            .forwardTicksToInches(.00199253807)
            .strafeTicksToInches(.002037534791)
            .turnTicksToInches(.00200683325)
            .leftPodY(112/25.4)
            .rightPodY(-118/25.4)
            .strafePodX(-126/25.4)
            .leftEncoder_HardwareMapName("FRwheel")
            .rightEncoder_HardwareMapName("BRwheel")
            .strafeEncoder_HardwareMapName("FLwheel")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.LEFT));


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
