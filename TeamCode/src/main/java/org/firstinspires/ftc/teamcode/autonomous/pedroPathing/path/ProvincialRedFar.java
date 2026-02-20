package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Autonomous
@Configurable // Panels
public class ProvincialRedFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    // ✅ timer that starts AFTER a path finishes
    private double pauseStartTime = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 9.5, Math.toRadians(90)));

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
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 9.500), new Pose(56.000, 13.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(112.5)
                    )
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(56.000, 13.000), new Pose(10.000, 10.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 10.000), new Pose(62.000, 18.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(112.5))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.000, 18.000), new Pose(62.000, 35.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(0))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(62.000, 35.000), new Pose(10.000, 35.000))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.000, 35.000), new Pose(62.000, 18.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(112.5))
                    .build();
        }
    }

    // ✅ helper: wait 1 second AFTER follower becomes not busy
    private boolean finishedAndPaused1s() {
        if (!follower.isBusy()) {
            if (pauseStartTime < 0) {
                pauseStartTime = getRuntime(); // start pause timer once
            }
            return (getRuntime() - pauseStartTime) >= 1.0;
        } else {
            pauseStartTime = -1; // reset timer while path is running
            return false;
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;

            case 1:
                if (finishedAndPaused1s()) {
                    pathState = 2;
                }
                break;

            case 2:
                follower.followPath(paths.Path2, true);
                pathState = 3;
                break;

            case 3:
                if (finishedAndPaused1s()) {
                    pathState = 4;
                }
                break;

            case 4:
                follower.followPath(paths.Path3, true);
                pathState = 5;
                break;

            case 5:
                if (finishedAndPaused1s()) {
                    pathState = 6;
                }
                break;

            case 6:
                follower.followPath(paths.Path4, true);
                pathState = 7;
                break;

            case 7:
                if (finishedAndPaused1s()) {
                    pathState = 8;
                }
                break;

            case 8:
                follower.followPath(paths.Path5, true);
                pathState = 9;
                break;

            case 9:
                if (finishedAndPaused1s()) {
                    pathState = 10;
                }
                break;

            case 10:
                follower.followPath(paths.Path6, true);
                pathState = 11;
                break;

            case 11:
                if(!follower.isBusy()){
                    break;
                }
        }

        return pathState;
    }
}