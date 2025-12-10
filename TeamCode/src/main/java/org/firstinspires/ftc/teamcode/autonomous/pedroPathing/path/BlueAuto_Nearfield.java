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
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.constants.Constants;

@Autonomous
@Configurable
public class BlueAuto_Nearfield extends OpMode {

    // hardware
    private DcMotor shooter1, shooter2, index;
    private DistanceSensor Distance;

    public Follower follower;
    private DcMotor IntakeMotor;

    private DigitalChannel Mag_Switch;

    private boolean limitSwitchState;
    private int ballCount = 0;

    // intake
    private double IntakePower =  1;

    //index
    private double autoIndexStartTime = 0;
    private boolean autoIndexing = false;
    private int indexActive = 0;

    // telemetry
    private TelemetryManager panelsTelemetry;

    // pp
    private Paths paths;
    private int pathState = 0;

    // shooter pid
    private double kP = 0.003, kI = 0.0015, kD = 0.0;
    private double shooterTargetTPS = 1100;
    private double NearShooterTargetTPS = 950;
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
        Distance = hardwareMap.get(DistanceSensor.class, "Distance1");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");


        Mag_Switch = hardwareMap.get(DigitalChannel.class, "Mag_Switch");
        Mag_Switch.setMode(DigitalChannel.Mode.INPUT);


        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

        // pp
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(21.308, 123.589, Math.toRadians(-138)));
        paths = new Paths(follower);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        double currentTime = getRuntime();
        limitSwitchState = !Mag_Switch.getState();
        double DistanceOn = (Distance.getDistance(DistanceUnit.CM));
        if (DistanceOn < 2.4 && !autoIndexing  && indexActive != 1){
            ballCount = ballCount + 1;
            autoIndexing = true;
            autoIndexStartTime = getRuntime();
            telemetry.addLine("The Ball Is Inside");
        }



        // shooter pid run continuously
        double shooterTPS = getShooterTPS();
        double shooterPower = shooterPID(shooterTPS);
        shooter1.setPower(shooterPower);
        shooter2.setPower(shooterPower);

        // state machine
        switch (autoState) {

            case 0:
                stateStartTime = currentTime;
                autoState = 1;
                break;


            case 1: // start pp
                follower.followPath(paths.Path1, true);
                autoState = 2;
                break;

            case 2: // wait until path finished
                if (!follower.isBusy()) {
                    autoState = 3; // done
                }
            case 3:
                shooter1.setPower(NearShooterTargetTPS);
                shooter2.setPower(NearShooterTargetTPS);

                if (currentTime - stateStartTime > 3) {
                    autoState = 4; // start pp
                }
                break;
            case 4: // spin up shooter and run indexer for 4 sec
                index.setPower(-0.4); // run indexer continuously
                shooter1.setPower(NearShooterTargetTPS);
                shooter2.setPower(NearShooterTargetTPS);
                if (currentTime - stateStartTime > 8) {
                    autoState = 5; // start pp
                }
                break;
            case 5: // start pp
                follower.followPath(paths.Path2, true);
                autoState = 6;
            case 6: // wait until path finished
                if (!follower.isBusy()) {
                    autoState = 7; // done
                }
            case 7:
                IntakeMotor.setPower(IntakePower);
                follower.followPath(paths.Path3, true);

                if (currentTime - stateStartTime > 13) {
                    autoState = 8;
                }
            case 8:
                if (!follower.isBusy()) {
                    autoState = 9;
                }

            case 9: // stop everything
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
        public PathChain Path2;
        public PathChain Path3;



        public Paths(Follower follower) {

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21.308, 124.037), new Pose(69.981, 73.794))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(135))
                    .build();
            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(69.981, 73.794), new Pose(37.009, 83.664))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(1))
                    .build();
            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(37.009, 83.664), new Pose(5.832, 83.664))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(1), Math.toRadians(1))
                    .build();
        }

    }
}