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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Autonomous
@Configurable
public class RedAuto_Farfield extends OpMode {

    // hardware
    private DcMotor shooter1, shooter2, index;
    public Follower follower;

    // telemetry
    private TelemetryManager panelsTelemetry;

    // pp
    private Paths paths;
    private int pathState = 0;

    // shooter pid
    private double kP = 0.003, kI = 0.0015, kD = 0.0;
    private double shooterTargetTPS = 1100;
    private int lastShooterPos = 0;
    private double lastTime = 0, shooterIntegral = 0, lastError = 0;

    // state machine
    private int autoState = 0;
    private double stateStartTime = 0;

    @Override
    public void init() {

        // telemetry
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // hardware
        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        index = hardwareMap.get(DcMotor.class, "Index");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

        // pp
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(67.5)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();

        // shooter pid run continuously
        double shooterTPS = getShooterTPS();
        double shooterPower = shooterPID(shooterTPS);
        shooter1.setPower(shooterPower);
        shooter2.setPower(shooterPower);

        // state machine
        switch (autoState) {

            case 0: // start shooter, wait 2 sec before indexer runs
                stateStartTime = currentTime;
                autoState = 1;
                break;

            case 1: //shooter run 3 second until the index is on
                index.setPower(0);
                if(currentTime - stateStartTime > 3.0){
                    autoState = 2;
                }
                break;

            case 2: // spin up shooter and run indexer for 4 sec
                index.setPower(-0.5); // run indexer continuously
                if (currentTime - stateStartTime > 7.0) { // wait 4 sec
                    autoState = 3; // start pp
                }
                break;

            case 3: // start pp
                follower.followPath(paths.Path1, true);
                autoState = 4;
                break;

            case 4: // wait until path finished
                if (!follower.isBusy()) {
                    autoState = 5; // done
                }
                break;

            case 5: // stop everything
                shooter1.setPower(0);
                shooter2.setPower(0);
                index.setPower(0);
                break;
        }

        // update pp follower
        follower.update();

        // telemetry
        panelsTelemetry.debug("Auto State", autoState);
        panelsTelemetry.debug("Shooter TPS", shooterTPS);
        panelsTelemetry.debug("Index Power", index.getPower());
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.update(telemetry);
    }

    // pid helpers
    private double getShooterTPS() {
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;

        double tps = 0;
        if (dt > 0)
            tps = (currentPos - lastShooterPos) / dt;

        lastShooterPos = currentPos;
        lastTime = currentTime;

        return Math.abs(tps);
    }

    private double shooterPID(double measuredTPS) {
        double error = shooterTargetTPS - measuredTPS;
        double dt = getRuntime() - lastTime;

        shooterIntegral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = kP * error + kI * shooterIntegral + kD * derivative;

        lastError = error;
        return -Range.clip(output, 0, 1);
    }

    // paths
    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(89, 12), new Pose(57, 12)))
                    .setLinearHeadingInterpolation(Math.toRadians(67.5), Math.toRadians(180))
                    .build();
        }

    }
}