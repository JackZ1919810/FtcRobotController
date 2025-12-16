package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

import java.util.List;

@Autonomous(name = "BoltonPathRedFar", group = "Autonomous")
@Configurable
public class BoltonPathRedFar extends OpMode {

    private TelemetryManager panelsTelemetry;

    public Follower follower;
    private Paths paths;

    private DcMotor FRwheel, BRwheel, FLwheel, BLwheel;
    private DcMotor shooter1, shooter2, IntakeMotor, Index;

    private Limelight3A limelight;

    // -------- Shooter PID --------
    private double kP = 0.003;
    private double kI = 0.0015;
    private double kD = 0.0;

    private double shooterTargetTPS = 1100;

    private int lastShooterPos = 0;
    private double lastShooterTime = 0;

    private double shooterIntegral = 0;
    private double lastError = 0;
    private double pidLastTime = 0;

    // -------- State machines --------
    private int pathState = 0;
    private int actionState = 0;  // 0 idle, 10 align, 20 spin, 30 feed, 40 done
    private double actionStartTime = 0;

    private double pathStartTime = 0;

    private int cycleCount = 0;
    private static final int MAX_CYCLES = 3;

    // -------- Limelight align stability fixes --------
    private double noTagSince = -1;
    private double alignedSince = -1;

    private static final int TAG_ID = 24;

    private static final double ALIGN_TOL_DEG = 2.0;
    private static final double ALIGN_HOLD_SEC = 0.20;
    private static final double NOTAG_GRACE_SEC = 0.25;
    private static final double SEARCH_TURN = 0.12;

    private static final double TURN_KP = 0.01;
    private static final double TURN_MAX = 0.30;
    private static final double ALIGN_TIMEOUT = 3.0; // seconds

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(59, 12, Math.toRadians(90)));
        paths = new Paths(follower);

        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        FRwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BRwheel.setDirection(DcMotorSimple.Direction.REVERSE);

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
        follower.update();
        double now = getRuntime();

        updatePathState(now);
        updateActionState(now);

        // Shooter PID if spinning/feeding
        double tps = getShooterTPS();
        boolean shooterOn = (actionState == 10 || actionState == 20 || actionState == 30);

        if (shooterOn) {
            double pwr = shooterPID(tps, shooterTargetTPS);
            shooter1.setPower(-pwr);
            shooter2.setPower(-pwr);
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            resetShooterPID();
        }

        // Debug
        panelsTelemetry.debug("PathState", pathState);
        panelsTelemetry.debug("ActionState", actionState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.debug("ShooterTPS", tps);
        panelsTelemetry.update(telemetry);
    }

    // ---------------------------
    // PATH STATE MACHINE
    // ---------------------------
    private void updatePathState(double now) {
        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                pathStartTime = now;
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()) {
                    pathState = 2;
                }
                break;

            case 2:
                startAlignMode(now);
                pathState = 3;
                break;

            case 3:
                if (actionState == 40) {
                    pathState = 4;
                }
                break;

            case 4:
                actionState = 0;
                follower.followPath(paths.Path2, false);

                IntakeMotor.setPower(1.0);
                Index.setPower(-0.30);
                pathStartTime = now;

                pathState = 5;
                break;

            case 5:
                if (!follower.isBusy()) {
                    if (now - pathStartTime >= 3.5){
                        IntakeMotor.setPower(0);
                        Index.setPower(0);
                        pathState = 6;
                    }
                }
                break;

            case 6:
                follower.followPath(paths.Path3, false);
                IntakeMotor.setPower(1.0);
                pathState = 7;
                break;

            case 7:
                if (!follower.isBusy()) {
                    IntakeMotor.setPower(0);
                    cycleCount++;

                    if (cycleCount >= MAX_CYCLES) {
                        pathState = 99;
                    } else {
                        startAlignMode(now);
                        pathState = 3;
                    }
                }
                break;

            case 99:
            default:
                IntakeMotor.setPower(0);
                Index.setPower(0);
                actionState = 0;
                break;
        }
    }

    private void startAlignMode(double now) {
        actionState = 10;
        noTagSince = -1;
        alignedSince = -1;
        actionStartTime = now;
    }

    // ---------------------------
    // ACTION STATE MACHINE
    // ---------------------------
    private void updateActionState(double now) {
        switch (actionState) {

            case 0:
                break;

            case 10: { // LIMELIGHT ALIGN

                double elapsed = getRuntime() - actionStartTime;

                // --- TIMEOUT CHECK ---
                if (elapsed > ALIGN_TIMEOUT) {
                    // Give up aligning, go collect
                    turnInPlace(0);
                    actionState = 20;   // treat as "done" so path can continue
                    break;
                }

                Double tx = getTxForTag24();

                if (tx == null) {
                    // Slow search
                    turnInPlace(0.12);
                }
                else if (!aligned(tx)) {
                    // Controlled correction
                    turnInPlace(alignTurnPower(tx));
                }
                else {
                    // SUCCESS
                    turnInPlace(0);
                    actionStartTime = getRuntime();
                    actionState = 20; // spin shooter
                    resetShooterPID();
                }
                break;
            }



            case 20: {
                if (now - actionStartTime >= 1.2) {
                    actionStartTime = now;
                    actionState = 30;
                }
                break;
            }

            case 30: {
                Index.setPower(-0.40);
                IntakeMotor.setPower(0.20);

                if (now - actionStartTime >= 3.5) {
                    Index.setPower(0);
                    IntakeMotor.setPower(0);
                    actionState = 40;
                }
                break;
            }

            case 40:
                Index.setPower(0);
                IntakeMotor.setPower(0);
                break;
        }
    }

    // ---------------------------
    // Limelight helper: Tag 20 only
    // - Choose best Tag20 detection using target area
    // - Return tx for that detection
    // ---------------------------
    private boolean aligned(double tx) {
        return Math.abs(tx) < 3; // tighter, stable threshold
    }
    private Double getTxForTag24() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult best = null;
        double bestArea = -1;

        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() != TAG_ID) continue;

            double area = f.getTargetArea(); // <-- your requested method name
            if (area > bestArea) {
                bestArea = area;
                best = f;
            }
        }

        if (best == null) return null;
        return best.getTargetXDegrees();
    }

    private double alignTurnPower(double tx) {
        if (Math.abs(tx) < ALIGN_TOL_DEG) return 0.0;

        double turn = tx * TURN_KP;
        return Range.clip(turn, -TURN_MAX, TURN_MAX);
    }

    private void turnInPlace(double turn) {
        FRwheel.setPower(-turn);
        BRwheel.setPower(-turn);
        FLwheel.setPower(turn);
        BLwheel.setPower(turn);
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

    // ---------------------------
    // Paths
    // ---------------------------
    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(59.000, 12.000), new Pose(59.000, 7.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(67.5))
                    .build();

            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(52.000, 7.000), new Pose(16.000, 12.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(180))
                    .build();

            Path3 = follower.pathBuilder()
                    .addPath(new BezierLine(new Pose(16.000, 12.000), new Pose(52.000, 7.000)))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(67.5))
                    .build();
        }
    }
}
