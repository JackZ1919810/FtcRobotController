package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Autonomous
@Configurable // Panels
public class ProvincialBlueFar extends OpMode {
    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private ProvincialBlueFar.Paths paths;

    private DcMotor FRwheel, BRwheel, FLwheel, BLwheel;
    private DcMotor shooter1, shooter2, IntakeMotor, Index;

    private Limelight3A limelight;

    // -------- Shooter PID --------
    private double kP = 0.003;
    private double kI = 0.0015;
    private double kD = 0.0;

    private double shooterTargetTPS = 500;

    private int lastShooterPos = 0;
    private double lastShooterTime = 0;

    private double shooterIntegral = 0;
    private double lastError = 0;
    private double pidLastTime = 0;

    // -------- State machines --------
    private int pathState = 0;
    private int actionState = 0;  // 0, 10, 20, 30, 40 done
    private double actionStartTime = 0;
    private double pathStartTime = 0;

    private int cycleCount = 0;
    private static final int MAX_CYCLES = 3;

    // ✅ timer that starts AFTER a path finishes
    private double pauseStartTime = -1;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 9.5, Math.toRadians(90)));
        paths = new ProvincialBlueFar.Paths(follower);

        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        FLwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BLwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Index = hardwareMap.get(DcMotor.class, "Index");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastShooterTime = getRuntime();
        pidLastTime = getRuntime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        double now = getRuntime();

        pathState = autonomousPathUpdate(now); // Update autonomous state machine
        updateActionState(now);

        // Shooter PID if spinning/feeding
        double tps = getShooterTPS();
        boolean shooterOn = (actionState == 20 || actionState == 30);

        if (shooterOn) {
            double pwr = shooterPID(tps, shooterTargetTPS);
            shooter1.setPower(-pwr);
            shooter2.setPower(-pwr);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            resetShooterPID();
        }

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

    // ---------------------------
    // Shooter TPS + PID
    // ---------------------------
    private double getShooterTPS() {
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastShooterTime;

        double tps = 0;
        if (dt > 0) {
            tps = (currentPos - lastShooterPos) / dt;
        }

        lastShooterPos = currentPos;
        lastShooterTime = currentTime;
        return tps;
    }

    private double shooterPID(double tps, double target) {
        double currentSpeed = Math.abs(tps);
        double error = target - currentSpeed;

        double now = getRuntime();
        double dt = Math.max(1e-3, now - pidLastTime);

        shooterIntegral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = kP * error + kI * shooterIntegral + kD * derivative;

        lastError = error;
        pidLastTime = now;

        return -Range.clip(-output, -1, 0);
    }

    private void resetShooterPID() {
        shooterIntegral = 0;
        lastError = 0;
        pidLastTime = getRuntime();
    }

    public int autonomousPathUpdate(double now) {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                pathState = 1;
                actionState = 10;
                break;

            case 1:
                if (finishedAndPaused1s() && actionState == 40) {
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

    private void updateActionState(double now) {
        switch (actionState) {
            case 10:
                actionStartTime = now;
                actionState = 20;
                break;

            case 20:
                if (now - actionStartTime >= 2){
                    actionState = 30;
                    actionStartTime = now;
                }
                break;

            case 30:
                Index.setPower(-1);
                if(now - actionStartTime >= 1.5){
                    actionState = 40;
                }
                break;

            case 40:
                Index.setPower(0);
                IntakeMotor.setPower(0);
                break;
        }
    }

}