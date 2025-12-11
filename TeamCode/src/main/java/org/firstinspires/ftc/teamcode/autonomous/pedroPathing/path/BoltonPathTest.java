package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class BoltonPathTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private double stateStartTime;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {

        follower.update(); // Update Pedro Pathing
        double currentTime = getRuntime();

        switch (pathState) {

            // ---- Path 1 ----
            case 0:
                // Start Path1 once (with reset = true if you want to snap to starting pose)
                stateStartTime = currentTime;
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;

            case 1:
                // Wait until Path1 finishes
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            // ---- Path 3 ----
            case 2:
                follower.followPath(paths.Path2, false);
                pathState = 3;
                break;

            case 3:
                if (!follower.isBusy()) {
                    pathState = 4;
                }
                break;

            // ---- Path 4 ----
            case 4:
                follower.followPath(paths.Path3, false);
                pathState = 5;
                break;

            case 5:
                if (!follower.isBusy()) {
                    pathState = 6;
                }
                break;

            // ---- Path 6 ----
            case 6:
                follower.followPath(paths.Path4, false);
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    pathState = 8;
                }
                break;

            // ---- Path 7 ----
            case 8:
                follower.followPath(paths.Path5, false);
                pathState = 9;
                break;

            case 9:
                if (!follower.isBusy()) {
                    // All paths finished
                    pathState = 10;
                }
                break;

            // ---- Done / Idle ----
            case 10:
            default:
                // Do nothing; robot just holds the final pose.
                break;
        }

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

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 12.000), new Pose(85.000, 7.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(90),
                            Math.toRadians(112.5)
                    )
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, 7.000), new Pose(85.000, -24.500))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000, -24.500), new Pose(105.000, -24.500))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(115.000, -24.500), new Pose(85.000, 7.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(112.5))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(85.000,7.000), new Pose(105.000, 7.000))
                    )
                    .setLinearHeadingInterpolation(
                            Math.toRadians(112.5),
                            Math.toRadians(90)
                    )
                    .build();
        }
    }

    public int autonomousPathUpdate() {


        return pathState;
    }

}
