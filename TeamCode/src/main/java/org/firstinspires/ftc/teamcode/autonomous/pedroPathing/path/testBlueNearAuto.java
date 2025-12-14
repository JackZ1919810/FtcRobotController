package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Disabled
@Autonomous
@Configurable // Panels
public class testBlueNearAuto extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(20, 121, Math.toRadians(140)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 121.000), new Pose(-13.000, 157.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(140))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(-13, 157.000), new Pose(28, 157.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            // ---- Path 1 ----
            case 0:
                // Start Path1 once (with reset = true if you want to snap to starting pose)
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;

            case 1:
                // Wait until Path1 finishes
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            case 2:
                follower.followPath(paths.Path2, false);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = 4;
                }
                break;

            case 4:
            default:
                break;
        }
        return pathState;
    }
}