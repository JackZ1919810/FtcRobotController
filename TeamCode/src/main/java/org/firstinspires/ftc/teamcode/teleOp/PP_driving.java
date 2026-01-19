package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@TeleOp(name = "Pedro TeleOp")
public class PP_driving extends OpMode {
    private Follower follower;
    private final Pose startPose = new Pose(0, 0, 0); // Starting position

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        // This tells the follower to ignore path following and listen to TeleOp vectors
        follower.startTeleOpDrive();
    }
    @Override
    public void loop() {
        // 1. Get Joystick Inputs
        double forwardInput = -gamepad1.left_stick_y;
        double strafeInput = gamepad1.left_stick_x;
        double turnInput = gamepad1.right_stick_x;

        // Deadband to prevent drift
        if (Math.abs(forwardInput) < 0.05) forwardInput = 0.0;
        if (Math.abs(strafeInput) < 0.05) strafeInput = 0.0;
        if (Math.abs(turnInput) < 0.05) turnInput = 0.0;

        // 2. Send the inputs to PedroPathing's built-in TeleOp drive
        //    robotCentric = false  => field-centric (Pedro handles the rotation internally)
        follower.setTeleOpDrive(forwardInput, strafeInput, turnInput, false);

        // 4. Update
        follower.update();

        // Telemetry
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }
}