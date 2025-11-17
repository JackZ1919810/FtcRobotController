package org.firstinspires.ftc.teamcode.pedroPathing;

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
            .forwardZeroPowerAcceleration(-61.93458886572208)
            .lateralZeroPowerAcceleration(-61.54349644482595)
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
            .xVelocity(61.01595852060964)
            .yVelocity(62.531513046113915);


    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            /**.forwardTicksToInches(-0.01434737309)
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
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
    **/

            .forwardTicksToInches(.00198690535)
            .strafeTicksToInches(.001976282225)
            .turnTicksToInches(.001999443333)
            .leftPodY(112/25.4)
            .rightPodY(-112/25.4)
            .strafePodX(-112/25.4)
            .leftEncoder_HardwareMapName("BLwheel")
            .rightEncoder_HardwareMapName("BRwheel")
            .strafeEncoder_HardwareMapName("shooter_right")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));


    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
