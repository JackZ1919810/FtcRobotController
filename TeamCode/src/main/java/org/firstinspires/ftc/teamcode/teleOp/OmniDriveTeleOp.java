package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

// Limelight imports
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.List;

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;

    // Limelight
    private Limelight3A limelight;

    // Magnetic switch (from TestTeleOp)
    private DigitalChannel Mag_Switch;
    private boolean limitSwitchState;

    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;

    // Auto-aim tuning gains
    private double kStrafe = 0.6;
    private double kForward = 0.8;
    private double kTurn = 0.02;

    // FAR shot target pose (fill with tuned values if you want auto-aim position control)
    private static final double FAR_DESIRED_X   = 0.0;
    private static final double FAR_DESIRED_Y   = 0.0;
    private static final double FAR_DESIRED_YAW = 0.0;

    // CLOSE shot target pose
    private static final double CLOSE_DESIRED_X   = 0.0;
    private static final double CLOSE_DESIRED_Y   = 0.0;
    private static final double CLOSE_DESIRED_YAW = 0.0;

    private double xTolerance = 0.03;
    private double yTolerance = 0.03;
    private double yawTolerance = 1.5;

    // Shooter PID (copied from TestTeleOp)
    private double kP = 0.0055;
    private double kI = 0.0000;
    private double kD = 0.0050;

    // Distance-based auto index flags (we keep this, but logic is same style as TestTeleOp)
    private boolean autoIndexing = false;
    private double autoIndexStartTime = 0;
    private int ballCount = 0;

    // Shooter TPS presets for far/close (OmniDriveTeleOp original)
    private static final double FAR_SHOOTER_TPS   = 2800.0;
    private static final double CLOSE_SHOOTER_TPS = 2400.0;

    private double shooterTargetTPS = CLOSE_SHOOTER_TPS;

    // Shooter PID state
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;

    // Extra shooter state from TestTeleOp
    private double startTime = 0;
    private int shooterLoopTimes = 0;

    private double IntakePower = 1;
    private double IndexPower = 0.30;   // from TestTeleOp
    private double stop = 0;

    private boolean reverseControls = false;
    private boolean lastAState = false;

    private IMU imu;

    // index state (from TestTeleOp)
    private int indexActive = 0;

    // Shot distance enum (OmniDriveTeleOp original)
    private enum ShotDistance {
        CLOSE,
        FAR
    }

    private ShotDistance lastShotDistance = ShotDistance.CLOSE;

    // Field-Centric Drive and Heading PID variables
    private double targetHeading = 0.0;
    private double heading_kP = 0.02;
    private double heading_kI = 0.0;
    private double heading_kD = 0.001;
    private double headingIntegral = 0.0;
    private double lastHeadingError = 0.0;
    private double headingTime = 0.0;

    private boolean autoAimActive = false;

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
        Distance = hardwareMap.get(DistanceSensor.class, "Distance1");

        // Mag switch like in TestTeleOp
        Mag_Switch = hardwareMap.get(DigitalChannel.class, "Mag_Switch");
        Mag_Switch.setMode(DigitalChannel.Mode.INPUT);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        FRwheel.setDirection(DcMotorSimple.Direction.REVERSE);
        BRwheel.setDirection(DcMotorSimple.Direction.REVERSE);

        FRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BRwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BLwheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        IntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Index.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Limelight init (from OmniDriveTeleOp)
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
        imu.resetYaw();
        targetHeading = 0;
        shooterTargetTPS = CLOSE_SHOOTER_TPS;
        lastShotDistance = ShotDistance.CLOSE;
    }

    @Override
    public void stop() {
        // limelight.stop(); // optional
    }

    @Override
    public void loop() {

        // ===== Sensors / Heading / Mag switch =====
        limitSwitchState = !Mag_Switch.getState(); // active-low like TestTeleOp
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double headingDeg = orientation.getYaw(AngleUnit.DEGREES);

        // ===== Gamepad 1: Drive + Auto-Aim =====
        boolean currentAState = gamepad1.a;
        if (currentAState && !lastAState) {
            reverseControls = !reverseControls;
        }
        lastAState = currentAState;

        boolean autoAimTrigger = gamepad1.right_trigger > 0.1;

        // Limelight AprilTag detection
        FiducialResult bestTag = null;
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            List<FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null) {
                for (FiducialResult fid : fiducials) {
                    int id = fid.getFiducialId();
                    if (id == 20 || id == 24) {
                        bestTag = fid;
                        break;
                    }
                }
            }
        }

        double lx, ly, rx;

        if (autoAimTrigger && bestTag != null) {
            // --- Auto-Aim logic (same as OmniDriveTeleOp) ---
            autoAimActive = true;
            Pose3D pose = bestTag.getRobotPoseTargetSpace();

            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);

            double targetX, targetY, targetYaw;
            if (lastShotDistance == ShotDistance.FAR) {
                targetX = FAR_DESIRED_X;
                targetY = FAR_DESIRED_Y;
                targetYaw = FAR_DESIRED_YAW;
            } else {
                targetX = CLOSE_DESIRED_X;
                targetY = CLOSE_DESIRED_Y;
                targetYaw = CLOSE_DESIRED_YAW;
            }

            double xError = targetX - x;
            double yError = targetY - y;
            double yawError = targetYaw - yaw;

            double strafeCmd =
                    (Math.abs(xError) > xTolerance) ? Range.clip(xError * kStrafe, -0.7, 0.7) : 0;
            double forwardCmd =
                    (Math.abs(yError) > yTolerance) ? Range.clip(yError * kForward, -0.7, 0.7) : 0;
            double turnCmd =
                    (Math.abs(yawError) > yawTolerance) ? Range.clip(yawError * kTurn, -0.5, 0.5) : 0;

            lx = strafeCmd;
            ly = forwardCmd;
            rx = turnCmd;

            telemetry.addData("TagID", bestTag.getFiducialId());
            telemetry.addData("LL RobotPose X", x);
            telemetry.addData("LL RobotPose Y", y);
            telemetry.addData("LL RobotPose Yaw", yaw);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.addData("Target Yaw", targetYaw);
            telemetry.addData("Error X", xError);
            telemetry.addData("Error Y", yError);
            telemetry.addData("Error Yaw", yawError);

        } else {
            // --- Manual field-centric driving with heading hold ---
            autoAimActive = false;

            double y_stick = applyDeadzone(Math.pow(-gamepad1.left_stick_y, 3));
            double x_stick = applyDeadzone(Math.pow(gamepad1.left_stick_x, 3));
            double turn_stick = applyDeadzone(Math.pow(gamepad1.right_stick_x, 3)) * TURN_SCALE;

            if (reverseControls) {
                x_stick = -x_stick;
                y_stick = -y_stick;
                turn_stick = -turn_stick;
            }

            double headingRadians = Math.toRadians(headingDeg);
            ly = y_stick * Math.cos(headingRadians) - x_stick * Math.sin(headingRadians);
            lx = y_stick * Math.sin(headingRadians) + x_stick * Math.cos(headingRadians);

            if (Math.abs(turn_stick) > DEADZONE) {
                rx = turn_stick;
                targetHeading = headingDeg;
                headingIntegral = 0;
                lastHeadingError = 0;
            } else {
                double dtHeading = getRuntime() - headingTime;
                double error = AngleUnit.normalizeDegrees(targetHeading - headingDeg);
                headingIntegral += error * dtHeading;
                double derivative = (dtHeading > 0) ? (error - lastHeadingError) / dtHeading : 0;

                rx = (heading_kP * error) + (heading_kI * headingIntegral) + (heading_kD * derivative);
                lastHeadingError = error;
            }
            headingTime = getRuntime();
        }

        // --- Wheel power calc ---
        double fl = ly + lx + rx;
        double fr = ly - lx - rx;
        double bl = ly - lx + rx;
        double br = ly + lx - rx;
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        FLwheel.setPower(fl);
        FRwheel.setPower(fr);
        BLwheel.setPower(bl);
        BRwheel.setPower(br);

        // ===== Gamepad 2: Shooter + Intake + Index (from TestTeleOp) =====
        boolean shooterActive = (gamepad2.right_trigger > 0.1);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean BallsOut = (gamepad2.left_bumper);
        double DistanceOn = Distance.getDistance(DistanceUnit.CM);
        telemetry.addData("Distance (cm)", DistanceOn);

        // Index A-button logic (auto run to mag switch)
        if (gamepad2.a) {
            indexActive = 1;
        }

        // Shooter mode (keep FAR/CLOSE logic from OmniDriveTeleOp)
        if (gamepad2.dpad_up) {
            lastShotDistance = ShotDistance.FAR;
            shooterTargetTPS = FAR_SHOOTER_TPS;
        } else if (gamepad2.dpad_down) {
            lastShotDistance = ShotDistance.CLOSE;
            shooterTargetTPS = CLOSE_SHOOTER_TPS;
        }
        shooterTargetTPS = Range.clip(shooterTargetTPS, 0, 5000);

        // BALLS OUT (same as TestTeleOp)
        if (BallsOut) {
            Index.setPower(IndexPower);
            IntakeMotor.setPower(-IntakePower);
        } else {
            Index.setPower(stop);
            IntakeMotor.setPower(stop);
        }

        // --- Shooter PID & timed index feed (from TestTeleOp, using shooter2 encoder) ---
        int currentPos = shooter2.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = 0;

        if (dt > 0) {
            shooterTPS = (currentPos - lastShooterPos) / dt;
        }

        if (shooterActive && !BallsOut) {

            if (shooterLoopTimes == 0) {
                startTime = getRuntime();
            }

            double currentSpeed = Math.abs(shooterTPS);
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

            // Note the -output and clip(-1,0) like in TestTeleOp
            double shooterPower = -Range.clip(-output, -1, 0);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            telemetry.addData("Shooter run time", getRuntime() - startTime);

            // after 1 second of spin-up, start feeding index
            if (getRuntime() - startTime >= 1.0) {
                IndexPower = 0.5;
                Index.setPower(-IndexPower);
            }

            lastError = error;
            shooterLoopTimes++;
        } else if (!shooterActive) {
            // Shooter off: reset
            IndexPower = 0.3;
            shooterLoopTimes = 0;
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterIntegral = 0;
            lastError = 0;
        }

        lastShooterPos = currentPos;
        lastTime = currentTime;

        // --- Index control with mag switch (from TestTeleOp) ---
        if (indexActive == 1 && !BallsOut && !shooterActive) {
            if (!limitSwitchState) {
                Index.setPower(-IndexPower);
            } else {
                Index.setPower(stop);
                indexActive = 0;
            }
        } else if (!shooterActive && !BallsOut && indexActive != 1) {
            Index.setPower(stop);
            indexActive = 0;
        }

        // Intake control (from TestTeleOp)
        if (intakeActive) {
            IntakeMotor.setPower(IntakePower);
        } else if (!BallsOut) {
            IntakeMotor.setPower(stop);
        }

        double IndexValue = Index.getCurrentPosition();

        // --- Telemetry ---
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Target TPS", shooterTargetTPS);
        telemetry.addData("ShooterActive", shooterActive);
        telemetry.addData("IndexEncoder", IndexValue);
        telemetry.addData("Heading (deg)", "%.1f", headingDeg);
        telemetry.addData("Magnetic switch state", limitSwitchState);
        telemetry.addData("Index State", indexActive);
        telemetry.addData("AutoAim", autoAimActive);
        telemetry.addData("Shot Mode", lastShotDistance.toString());

        // Distance-based ball detect (same style as TestTeleOp; auto-index body can be added later)
        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && indexActive != 1) {
            ballCount++;
            autoIndexing = true;
            autoIndexStartTime = getRuntime();
            telemetry.addLine("The Ball Is Inside");
        }
        // (autoIndexing behavior can be implemented here if you want it active)

        telemetry.update();
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
