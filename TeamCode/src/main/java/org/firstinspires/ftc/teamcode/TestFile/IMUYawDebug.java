package org.firstinspires.ftc.teamcode.TestFile;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "IMU Yaw Debug", group = "Debug")
public class IMUYawDebug extends OpMode {

    private IMU imu;
    private boolean lastA = false;

    @Override
    public void init() {
        imu = hardwareMap.get(IMU.class, "imu");

        // IMPORTANT: this must match your Pedro localizer IMU_Orientation
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
        );

        imu.initialize(params);

        telemetry.addLine("IMU initialized. Align robot with field, then press A to zero yaw.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Edge-detect A button to reset yaw
        boolean currentA = gamepad1.a;
        if (currentA && !lastA) {
            imu.resetYaw();
        }
        lastA = currentA;

        // Read yaw/pitch/roll
        YawPitchRollAngles ang = imu.getRobotYawPitchRollAngles();
        double yawRad = ang.getYaw(AngleUnit.RADIANS);
        double yawDeg = ang.getYaw(AngleUnit.DEGREES);

        telemetry.addLine("Press A to set current direction as yaw = 0");
        telemetry.addData("Yaw (deg)", "%.2f", yawDeg);
        telemetry.addData("Yaw (rad)", "%.4f", yawRad);
        telemetry.addData("Pitch (deg)", "%.2f", ang.getPitch(AngleUnit.DEGREES));
        telemetry.addData("Roll (deg)", "%.2f", ang.getRoll(AngleUnit.DEGREES));
        telemetry.update();
    }
}
