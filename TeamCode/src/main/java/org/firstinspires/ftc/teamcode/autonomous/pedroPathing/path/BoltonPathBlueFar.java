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

@Autonomous
@Configurable // Panels
public class BoltonPathBlueFar extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private double stateStartTime;

    private DcMotor shooter1, shooter2, IntakeMotor, Index;

    private DcMotor FRwheel, BRwheel, FLwheel, BLwheel;

    private Limelight3A limelight;


    // PID vars (same idea as realTeleOp)
    private double kP = 0.003, kI = 0.0015, kD = 0;
    private double shooterTargetTPS = 1100;
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;

    // State machines
    private int actionState = 0; // 0 idle, 10 align, 20 spin, 30 feed, 40 done
    private double actionStartTime = 0;
    private double pathStartTime = 0;

    // Optional cycle control
    private int cycleCount = 0;
    private static final int MAX_CYCLES = 3; // or time based

    private double alignedSince = -1;
    private double tagSeenSince = -1;

    private double txFiltered = 0;
    private boolean txFilterInit = false;
    private double lastTagSeenTime = -1;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(85, 12, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Index = hardwareMap.get(DcMotor.class, "Index");

        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        FRwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BRwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();


    }

    @Override
    public void loop() {

        follower.update();
        double now = getRuntime();

        updatePathState(now);

        updateActionState(now);

        //shooter setting
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

        // Logging
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
                            new BezierLine(new Pose(85.000, 7.000), new Pose(126.000, 12.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(112.5), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(126.000, 12.000), new Pose(85.000, 7.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(112.5))
                    .build();
        }
    }

    private void updateActionState(double now) {
        switch (actionState) {

            case 0: {// idle
                // subsystems off by default
                break;
                }
            case 10: {
                Double tx = getBestTxForTags(20);

                if (tx == null) {
                    // If we saw a tag very recently, keep using the last filtered value briefly
                    if (lastTagSeenTime > 0 && now - lastTagSeenTime < 0.25) {
                        tx = txFiltered;
                    } else {
                        txFilterInit = false;
                        turnInPlace(0.12);
                        break;
                    }
                } else {
                    lastTagSeenTime = now;

                    // Low-pass filter (reduces jitter)
                    if (!txFilterInit) {
                        txFiltered = tx;
                        txFilterInit = true;
                    } else {
                        double alpha = 0.25; // 0.15–0.35 (smaller = smoother)
                        txFiltered = (1 - alpha) * txFiltered + alpha * tx;
                    }
                    tx = txFiltered;
                }

                if (!aligned(tx)) {
                    turnInPlace(alignTurnPower(tx));
                } else {
                    turnInPlace(0);
                    actionStartTime = now;
                    actionState = 20;
                    resetShooterPID();
                }
                telemetry.addData("LL valid", limelight.getLatestResult() != null && limelight.getLatestResult().isValid());
                telemetry.addData("tx raw", tx);

                break;

            }


            case 20: { // spin shooter
                if (now - actionStartTime >= 1.2) { // spin up time
                    actionStartTime = now;
                    actionState = 30;
                }
                break;
            }

            case 30: { // Index
                Index.setPower(-0.4); // feed
                IntakeMotor.setPower(0.5); // intake assist in case balls are stuck

                if (now - actionStartTime >= 3.5) { // feed time
                    Index.setPower(0);
                    IntakeMotor.setPower(0);
                    actionState = 40;
                }
                break;
            }

            case 40: // DONE
                Index.setPower(0);
                IntakeMotor.setPower(0);
                break;
        }
    }
    private void updatePathState(double now) {

        switch (pathState) {

            case 0:
                follower.followPath(paths.Path1, true);
                pathState = 1;
                break;

            case 1:
                if (!follower.isBusy()){
                    pathState = 2;
                }
                break;

            case 2: // start shoot sequence
                actionState = 10;      // start align
                alignedSince = -1;
                tagSeenSince = -1;
                pathState = 3;
                break;

            case 3: // wait until shoot done
                if (actionState == 40) {
                    pathState = 4;
                }
                break;

            case 4: // go collect (Path2)
                actionState = 0; // stop action machine
                follower.followPath(paths.Path2, false);
                IntakeMotor.setPower(1.0);   // start intake
                Index.setPower(-0.3);
                pathState = 5;
                pathStartTime = now;
                break;

            case 5: // wait Path2 finish
                if (!follower.isBusy()) {
                    if (now - pathStartTime >= 4.0) {
                        pathStartTime = now;
                        IntakeMotor.setPower(0);
                        Index.setPower(0);
                        pathState = 6;
                    }
                }
                break;

            case 6: // return (Path3)
                follower.followPath(paths.Path3, false);
                IntakeMotor.setPower(1.0); // keep collecting on return if you want
                pathState = 7;
                pathStartTime = now;
                break;

            case 7: // wait Path3 finish
                if (!follower.isBusy()) {
                    IntakeMotor.setPower(0);
                    cycleCount++;

                    if (cycleCount >= MAX_CYCLES) {
                        pathState = 99;  // done
                    } else {
                        if (now - pathStartTime >= 1){
                            // shoot again
                            actionState = 10;
                            alignedSince = -1;
                            tagSeenSince = -1;
                            pathState = 1;
                        }
                    }
                }
                break;

            case 99:
            default:
                // stop all
                break;
        }
    }

    private double getShooterTPS(){
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;

        double tps = 0;
        if (dt > 0) tps = (currentPos - lastShooterPos) / dt;

        lastShooterPos = currentPos;
        lastTime = currentTime;
        return tps;
    }

    private double shooterPID(double tps, double target){
        double currentSpeed = Math.abs(tps);
        double error = target - currentSpeed;

        double now = getRuntime();
        double dt = Math.max(1e-3,now-pidLastTime);

        shooterIntegral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = kP * error + kI * shooterIntegral + kD * derivative;

        lastError = error;
        pidLastTime = now;

        // keep sign convention consistent with your robot
        return -Range.clip(-output, -1, 0);
    }
    private double pidLastTime = 0;

    private void resetShooterPID(){
        shooterIntegral = 0;
        lastError = 0;
        pidLastTime = getRuntime();
    }

    private Double getBestTxForTags(int tagA) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        LLResultTypes.FiducialResult best = null;
        double bestScore = -1;

        for (LLResultTypes.FiducialResult f : fiducials) {
            int id = f.getFiducialId();
            if (id != tagA) continue;

            // Prefer "closer" tag: use ta (area) because pose can be null.
            // Bigger ta => closer in view (usually).
            double ta = f.getTargetArea();   // if your SDK uses getTA() instead, change it

            if (ta > bestScore) {
                bestScore = ta;
                best = f;
            }
        }

        if (best == null) return null;

        // IMPORTANT: use Limelight’s built-in tx, not atan2(x,z)
        return best.getTargetXDegrees();
    }


    private double alignTurnPower(double tx) {
        double kPturn = 0.01;
        double minTurn = 0.08;   // NEW: overcome static friction

        double turn = tx * kPturn;

        if (Math.abs(tx) > 2.0) {
            turn += Math.signum(turn) * minTurn;
        }

        return Range.clip(turn, -0.30, 0.30);
    }

    private boolean aligned(double tx) {
        return Math.abs(tx) < 2.0; // degrees tolerance
    }

    private void turnInPlace(double turn) {
        // mecanum turn-only pattern
        FRwheel.setPower(-turn);
        BRwheel.setPower(-turn);
        FLwheel.setPower(turn);
        BLwheel.setPower(turn);
    }

}
