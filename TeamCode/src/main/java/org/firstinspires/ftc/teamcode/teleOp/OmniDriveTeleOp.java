package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;
    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private boolean autoAimActive = false;

    private double kStrafe = 0.6;
    private double kForward = 0.8;
    private double kTurn = 0.02;

    private double desiredX = 0.0;
    private double desiredY = 0.65; // Renamed from desiredZ, Y is forward in FTC coordinates
    private double desiredYaw = 0.0;

    private double xTolerance = 0.03;
    private double yTolerance = 0.03; // Renamed from zTolerance
    private double yawTolerance = 1.5;

    private double kP = 0.0004;
    private double kI = 0.0000;
    private double kD = 0.0000;

    private boolean autoIndexing = false;
    private double autoIndexStartTime = 0;

    private double shooterTargetTPS = 2600.0;

    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;

    private double IntakePower =  1;
    private double IndexPower = 0.24;
    private double stop = 0;

    private boolean reverseControls = false;
    private boolean lastAState = false;

    private IMU imu;

    // Field-Centric Drive and Heading PID variables
    private double targetHeading = 0.0;
    private double heading_kP = 0.02; // Proportional gain for heading correction
    private double heading_kI = 0.0;     // Integral gain
    private double heading_kD = 0.001;   // Derivative gain
    private double headingIntegral = 0.0;
    private double lastHeadingError = 0.0;
    private double headingTime = 0.0;


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

        initAprilTag();

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
        imu.resetYaw(); // Reset yaw at the start
        targetHeading = 0;
    }

    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().build();

        visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(aprilTag)
            .build();
    }

    @Override
    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double headingDeg = orientation.getYaw(AngleUnit.DEGREES);

        // --- Gamepad 1 Controls ---

        boolean currentAState = gamepad1.a;
        if (currentAState && !lastAState) {
            reverseControls = !reverseControls;
        }
        lastAState = currentAState;

        boolean autoAimTrigger = gamepad1.right_trigger > 0.1;

        AprilTagDetection bestTag = null;
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null && (detection.id == 20 || detection.id == 24)) {
                bestTag = detection;
                break;
            }
        }

        double lx, ly, rx; // These will hold the final drive commands

        if (autoAimTrigger && bestTag != null) {
            // --- Auto-Aim Logic ---
            autoAimActive = true;
            double x = bestTag.ftcPose.x;
            double y = bestTag.ftcPose.y;
            double yaw = bestTag.ftcPose.yaw;

            double xError = desiredX - x;
            double yError = desiredY - y;
            double yawError = desiredYaw - yaw;

            double strafeCmd = (Math.abs(xError) > xTolerance) ? Range.clip(xError * kStrafe, -0.7, 0.7) : 0;
            double forwardCmd = (Math.abs(yError) > yTolerance) ? Range.clip(yError * kForward, -0.7, 0.7) : 0;
            double turnCmd = (Math.abs(yawError) > yawTolerance) ? Range.clip(yawError * kTurn, -0.5, 0.5) : 0;

            lx = strafeCmd;
            ly = forwardCmd;
            rx = turnCmd;

            telemetry.addData("TagID", bestTag.id);
            telemetry.addData("PoseX", x);
            telemetry.addData("PoseY", y);
            telemetry.addData("PoseYaw", yaw);
        } else {
            // --- Manual Driving Logic (with Field-Centric and Heading PID) ---
            autoAimActive = false;
            double y_stick = applyDeadzone(Math.pow(-gamepad1.left_stick_y, 3));
            double x_stick = applyDeadzone(Math.pow(gamepad1.left_stick_x, 3));
            double turn_stick = applyDeadzone(Math.pow(gamepad1.right_stick_x, 3)) * TURN_SCALE;

            if (reverseControls) {
                x_stick = -x_stick;
                y_stick = -y_stick;
                turn_stick = -turn_stick;
            }

            // Rotate the movement vector by the robot's heading
            double headingRadians = Math.toRadians(headingDeg);
            ly = y_stick * Math.cos(headingRadians) - x_stick * Math.sin(headingRadians);
            lx = y_stick * Math.sin(headingRadians) + x_stick * Math.cos(headingRadians);
            

            // Heading PID Controller
            if (Math.abs(turn_stick) > DEADZONE) {
                // Driver is manually turning
                rx = turn_stick;
                targetHeading = headingDeg; // Update the target to the current heading
                headingIntegral = 0; // Reset PID state
                lastHeadingError = 0;
            } else {
                // Driver is not turning, engage PID to hold heading
                double dt = getRuntime() - headingTime;
                double error = AngleUnit.normalizeDegrees(targetHeading - headingDeg);

                headingIntegral += error * dt;
                double derivative = (dt > 0) ? (error - lastHeadingError) / dt : 0;

                rx = (heading_kP * error) + (heading_kI * headingIntegral) + (heading_kD * derivative);
                lastHeadingError = error;
            }
            headingTime = getRuntime();
        }

        // --- Wheel Power Calculation ---
        double fl = ly + lx + rx;
        double fr = ly - lx - rx;
        double bl = ly - lx + rx;
        double br = ly + lx - rx;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));
        fl /= max;
        fr /= max;
        bl /= max;
        br /= max;

        FLwheel.setPower(fl);
        FRwheel.setPower(fr);
        BLwheel.setPower(bl);
        BRwheel.setPower(br);

        // --- Gamepad 2 Controls ---
        boolean shooterActive = (gamepad2.right_trigger > 0.1);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean IndexActive = (gamepad2.right_bumper);
        boolean BallsOut = (gamepad2.left_bumper);
        double DistanceOn = (Distance.getDistance(DistanceUnit.CM));

        telemetry.addData("Distance is ", Distance.getDistance(DistanceUnit.CM) );

        if (gamepad2.dpad_up) {
            shooterTargetTPS = 2600;
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 2200;
        }
        shooterTargetTPS = Range.clip(shooterTargetTPS, 0, 5000);

        if (BallsOut) {
            Index.setPower(IndexPower);
            IntakeMotor.setPower(-IntakePower);
        } else {
            Index.setPower(stop);
            IntakeMotor.setPower(stop);
        }

        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = 0;

        if (dt > 0) {
            shooterTPS = (currentPos - lastShooterPos) / dt;
        }

        if (shooterActive) {
            double currentSpeed = Math.abs(shooterTPS);
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

            double shooterPower = -Range.clip(output, 0, 1);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            lastError = error;
        } else {
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterIntegral = 0;
            lastError = 0;
        }

        lastShooterPos = currentPos;
        lastTime = currentTime;

        if (IndexActive) {
            Index.setPower(-IndexPower);
        } else if (!shooterActive && !BallsOut && !intakeActive) {
            Index.setPower(stop);
        }

        if (intakeActive) {
            IntakeMotor.setPower(IntakePower);
        } else if (!BallsOut) {
            IntakeMotor.setPower(stop);
        }

        double IndexValue = Index.getCurrentPosition();

        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Target TPS", shooterTargetTPS);
        telemetry.addData("ShooterActive", shooterActive);
        telemetry.addData("IndexEncoder:", IndexValue);
        telemetry.addData("Heading (deg)", "%.1f", headingDeg);
        telemetry.addData("AutoAim", autoAimActive);

        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && !IndexActive) {
            autoIndexing = true;
            autoIndexStartTime = getRuntime();
            telemetry.addLine("The Ball Is Inside");
        }

        if (autoIndexing) {
            if (BallsOut || IndexActive) {
                autoIndexing = false;
            } else {
                Index.setPower(-IndexPower);

                if (getRuntime() - autoIndexStartTime >= 0.3) {
                    Index.setPower(stop);
                    autoIndexing = false;
                }
            }
        }

        telemetry.update();
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
