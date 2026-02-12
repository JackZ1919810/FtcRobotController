package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class realTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;
    private DistanceSensor IntakeSensor;
    private DigitalChannel Mag_Switch;
    private IMU imu;
    private Limelight3A limelight;

    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;
    private static final double MAX_DRIVE_POWER = 0.85; // Cap drive power to save battery

    private boolean intakeDetectActive = false;
    private double intakeDetectStartTime = 0;
    private static final double INTAKE_DETECT_DISTANCE_M = 2;
    private static final double INTAKE_RUN_TIME_S = 4.0;

    private double IntakePower = 1.0;
    private double IndexPower = 1;
    private double stop = 0.0;

    private boolean autoAlignEnabled = false;
    private boolean lastAlignButton = false;
    private static final double kP_LL_Turn = 0.01;
    private static final double Max_LL_Turn = 0.5;
    private static final double LL_Angle_Range_DEG = 2.0;

    private double kP = 0.003;
    private double kI = 0.0015;
    private double kD = 0.000;
    private double shooterTargetTPS = 1100.0;
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;
    private double startTime = 0;
    private int shooterLoopTimes = 0;

    private boolean autoIndexing = false;
    private int indexActive = 0;
    private int ballCount = 0;
    private boolean reverseControls = false;
    private boolean lastAState = false;

    private int loopCounter = 0;

    // Store last power values to reduce unnecessary updates
    private double lastFLPower = 0;
    private double lastFRPower = 0;
    private double lastBLPower = 0;
    private double lastBRPower = 0;

    @Override
    public void init() {
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Index = hardwareMap.get(DcMotor.class, "Index");
        IntakeSensor = hardwareMap.get(DistanceSensor.class, "IntakeSensor");
        Mag_Switch = hardwareMap.get(DigitalChannel.class, "Mag_Switch");
        imu = hardwareMap.get(IMU.class, "imu");

        Mag_Switch.setMode(DigitalChannel.Mode.INPUT);

        FRwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BRwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        FRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(30);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    @Override
    public void loop() {
        loopCounter++;
        boolean updateLimelight = (loopCounter % 3 == 0);

        boolean shooterActive = (gamepad2.right_trigger > 0.1);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean ballsOut = gamepad2.left_bumper;
        boolean alignButton = gamepad1.right_bumper;
        boolean currentAState = gamepad1.a;

        if (currentAState && !lastAState) {
            reverseControls = !reverseControls;
        }
        lastAState = currentAState;

        if (alignButton && !lastAlignButton) {
            autoAlignEnabled = !autoAlignEnabled;
        }
        lastAlignButton = alignButton;

        double lx = cubicDeadzone(gamepad1.left_stick_x);
        double ly = cubicDeadzone(-gamepad1.left_stick_y);
        double rx = cubicDeadzone(gamepad1.right_stick_x) * TURN_SCALE;

        if (reverseControls) {
            lx = -lx;
            ly = -ly;
            rx = -rx;
        }


       // limelight align

        if (autoAlignEnabled && updateLimelight) {
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<FiducialResult> fiducials = result.getFiducialResults();
                FiducialResult closestTag = null;
                double closestZ = Double.MAX_VALUE;
                for (FiducialResult fid : fiducials) {
                    int id = fid.getFiducialId();
                    if (id == 20 || id == 24) {
                        Pose3D pose = fid.getTargetPoseCameraSpace();
                        if (pose != null && pose.getPosition().z < closestZ) {
                            closestZ = pose.getPosition().z;
                            closestTag = fid;
                        }
                    }
                }

                if (closestTag != null) {
                    double x = closestTag.getTargetPoseCameraSpace().getPosition().x;
                    double z = closestTag.getTargetPoseCameraSpace().getPosition().z;
                    double txDeg = Math.toDegrees(Math.atan2(x, z));
                    double turnCmd = txDeg * kP_LL_Turn;
                    turnCmd = Range.clip(turnCmd, -Max_LL_Turn, Max_LL_Turn);
                    if (Math.abs(txDeg) < LL_Angle_Range_DEG) turnCmd = 0.0;
                    rx = turnCmd;
                }
            }
        }

        double fl = ly + lx + rx;
        double fr = ly - lx - rx;
        double bl = ly - lx + rx;
        double br = ly + lx - rx;

        double max = Math.max(1.0, maxAbs(fl, fr, bl, br));
        fl /= max; fr /= max; bl /= max; br /= max;

        //  Cap maximum drive power to save battery
        fl = Range.clip(fl, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        fr = Range.clip(fr, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        bl = Range.clip(bl, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);
        br = Range.clip(br, -MAX_DRIVE_POWER, MAX_DRIVE_POWER);

        setMotorPowerOptimized(FLwheel, fl, lastFLPower);
        setMotorPowerOptimized(FRwheel, fr, lastFRPower);
        setMotorPowerOptimized(BLwheel, bl, lastBLPower);
        setMotorPowerOptimized(BRwheel, br, lastBRPower);

        lastFLPower = fl;
        lastFRPower = fr;
        lastBLPower = bl;
        lastBRPower = br;

        // shooter PID
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = dt > 0 ? (currentPos - lastShooterPos) / dt : 0;

        if (shooterActive && !ballsOut) {
            if (shooterLoopTimes == 0) startTime = getRuntime();
            indexActive = 0;
            double error = shooterTargetTPS - Math.abs(shooterTPS);
            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;
            double output = kP * error + kI * shooterIntegral + kD * derivative;
            double shooterPower = -Range.clip(-output, -1, 0);
            shooter1.setPower(-shooterPower);
            shooter2.setPower(-shooterPower);

            if (getRuntime() - startTime >= 1.5) {
                Index.setPower(-IndexPower);
            }

            lastError = error;
            shooterLoopTimes++;
        } else {
            shooterLoopTimes = 0;
            if (shooter1.getPower() != 0 || shooter2.getPower() != 0) {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }
            shooterIntegral = 0;
            lastError = 0;
        }

        lastShooterPos = currentPos;
        lastTime = currentTime;


        // turn on the distance detector when intake is active
        if (intakeActive) {
            double intakeDist = IntakeSensor.getDistance(DistanceUnit.METER);
            if (!intakeDetectActive && intakeDist > 0 && intakeDist < INTAKE_DETECT_DISTANCE_M) {
                intakeDetectActive = true;
                intakeDetectStartTime = getRuntime();
            }
        }

        // auto intake
        if (intakeDetectActive) {
            IntakeMotor.setPower(1.0);
            if (getRuntime() - intakeDetectStartTime >= INTAKE_RUN_TIME_S && !intakeActive) {
                intakeDetectActive = false;
                IntakeMotor.setPower(stop);
            }
        // normal intake

        } else if (intakeActive) {
            IntakeMotor.setPower(1.0);
        } else if (!ballsOut) {
            if (IntakeMotor.getPower() != stop) {
                IntakeMotor.setPower(stop);
            }
        }


        // balls out
        if (ballsOut) {
            Index.setPower(IndexPower);
            IntakeMotor.setPower(-1);
        } else if (!shooterActive && indexActive != 1) {
            if (Index.getPower() != stop) {
                Index.setPower(stop);
            }
        }


        // index
        if (gamepad2.a) indexActive = 1;
        boolean limitSwitchTriggered = !Mag_Switch.getState();
        if (indexActive == 1 && !ballsOut && !shooterActive) {
            if (!limitSwitchTriggered) {
                Index.setPower(-IndexPower);
                if (gamepad2.b) {
                    Index.setPower(stop);
                    indexActive = 0;
                }
            } else {
                Index.setPower(stop);
                indexActive = 0;
            }
        }

        // telemetry.addData("Shooter TPS", shooterTPS);
        // telemetry.addData("Loop Time (ms)", (System.nanoTime() / 1_000_000.0));
        // telemetry.update();
    }

    private static double cubicDeadzone(double input) {
        double value = input * input * input;
        return (Math.abs(value) < DEADZONE) ? 0.0 : value;
    }

    private double maxAbs(double... values) {
        double max = 0.0;
        for (double v : values) {
            max = Math.max(max, Math.abs(v));
        }
        return max;
    }

    private void setMotorPowerOptimized(DcMotor motor, double newPower, double lastPower) {
        if (Math.abs(newPower - lastPower) > 0.02) {
            motor.setPower(newPower);
        }
    }

}