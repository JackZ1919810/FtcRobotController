package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
<<<<<<< HEAD
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
=======

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
>>>>>>> parent of 163ddd0 (k)

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;
<<<<<<< HEAD

    private DigitalChannel Mag_Switch;

    private boolean limitSwitchState;
    private boolean indexDelayActive;
    private double indexDelayStart;
    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;

    // Shooter PID constants (start with these and tune)
    private double kP = 0.0055;
=======
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
>>>>>>> parent of 163ddd0 (k)
    private double kI = 0.0000;
    private double kD = 0.0050;

    // Auto index (distance-based) state
    private boolean autoIndexing = false;
    private int indexActive = 0;

    private int ballCount = 0;
    private double autoIndexStartTime = 0;

<<<<<<< HEAD

    // Target shooter speed in ticks per second (tune this!)
    private double shooterTargetTPS = 1000.0;
=======
    private double shooterTargetTPS = 2600.0;
>>>>>>> parent of 163ddd0 (k)

    // Shooter PID state
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;
    private double startTime = 0;
    private int shooterLoopTimes = 0;

    private double IntakePower =  1;
<<<<<<< HEAD
    private double IndexPower = 0.30;
=======
    private double IndexPower = 0.24;
>>>>>>> parent of 163ddd0 (k)
    private double stop = 0;

    private boolean reverseControls = false;
    private boolean lastAState = false;

    private IMU imu;

<<<<<<< HEAD
=======
    // Field-Centric Drive and Heading PID variables
    private double targetHeading = 0.0;
    private double heading_kP = 0.02; // Proportional gain for heading correction
    private double heading_kI = 0.0;     // Integral gain
    private double heading_kD = 0.001;   // Derivative gain
    private double headingIntegral = 0.0;
    private double lastHeadingError = 0.0;
    private double headingTime = 0.0;

>>>>>>> parent of 163ddd0 (k)

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

// setMode(DcMotor.RunMode.RUN_USING_ENCODER)

// Enable encoders for shooter PID
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

<<<<<<< HEAD
=======
        initAprilTag();
>>>>>>> parent of 163ddd0 (k)

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();

//Imu reading init
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters params = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(params);
<<<<<<< HEAD

=======
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
>>>>>>> parent of 163ddd0 (k)
    }

    @Override
    public void loop() {

        //get states
        limitSwitchState = !Mag_Switch.getState();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        double headingDeg = orientation.getYaw(AngleUnit.DEGREES);

// ========== DRIVE CONTROLS ==========
        boolean currentAState = gamepad1.a;
        if (currentAState && !lastAState) {
            reverseControls = !reverseControls;
        }
        lastAState = currentAState;

        double lx = applyDeadzone(Math.pow(gamepad1.left_stick_x, 3));
        double ly = applyDeadzone(Math.pow(-gamepad1.left_stick_y, 3));
        double rx = applyDeadzone(Math.pow(gamepad1.right_stick_x, 3)) * TURN_SCALE;

<<<<<<< HEAD
        if (reverseControls) {
            lx = -lx;
            ly = -ly;
            rx = -rx;
        }

=======
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
>>>>>>> parent of 163ddd0 (k)
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

//  GAMEPAD buttons
        boolean shooterActive = (gamepad2.right_trigger > 0.1);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean BallsOut = (gamepad2.left_bumper);

        //Index Gamepad logic
        if(gamepad2.a){
            indexActive = 1;
        }

        double DistanceOn = (Distance.getDistance(DistanceUnit.CM));

        telemetry.addData("Distance is ", Distance.getDistance(DistanceUnit.CM) );

<<<<<<< HEAD
//  adjust shooter target speed with dpad
        if (gamepad2.dpad_up) {
            shooterTargetTPS = 1100; // Far target
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 1000; // close target
=======
        if (gamepad2.dpad_up) {
            shooterTargetTPS = 2800; // Far target
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 2400; // close target

            shooterTargetTPS = 2600;
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 2200;
>>>>>>> parent of 163ddd0 (k)
        }
        shooterTargetTPS = Range.clip(shooterTargetTPS, 0, 5000); // clamp reasonable range

// BALLS OUT
        if (BallsOut) {
            Index.setPower(IndexPower);
            IntakeMotor.setPower(-IntakePower);
        } else {
            Index.setPower(stop);
            IntakeMotor.setPower(stop);
        }



//  SHOOTER PID CONTROL
// Measure shooter speed
        int currentPos = shooter2.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = 0;


        if (dt > 0) {
            shooterTPS = (currentPos - lastShooterPos) / dt; // ticks per second
        }

        if (shooterActive && !BallsOut) {

            if (shooterLoopTimes == 0){
                startTime = getRuntime();
            }
            double currentSpeed = Math.abs(shooterTPS); // PID on absolute speed so direction sign doesn't matter
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

// Shooter power command
            double shooterPower = -Range.clip(-output, -1, 0);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            telemetry.addData("shooter run time", getRuntime()-startTime);

            if (getRuntime()-startTime >= 1.0){
                IndexPower = 0.5;
                Index.setPower(-IndexPower);
            }

            lastError = error;
            shooterLoopTimes = shooterLoopTimes + 1;
        }
        else if (!shooterActive){
// Shooter off, reset PID state
            IndexPower = 0.3;
            shooterLoopTimes = 0;
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterIntegral = 0;
            lastError = 0;
        }

// Update last encoder/time for next loop
        lastShooterPos = currentPos;
        lastTime = currentTime;

// Index
      if (indexActive == 1 && !BallsOut && !shooterActive) {
            if (!limitSwitchState){
                Index.setPower(-IndexPower);
            }
            else{
                Index.setPower(stop);
                indexActive = 0;
            }
        }
        else if (!shooterActive && !BallsOut && indexActive != 1) {
// Only stop here if nothing else is trying to move Index
            Index.setPower(stop);
            indexActive = 0;
        }

// Intake
        if (intakeActive) {
            IntakeMotor.setPower(IntakePower);
        } else if (!BallsOut) {
            IntakeMotor.setPower(stop);
        }

        double IndexValue = Index.getCurrentPosition();

//  telemetry to help tune PID
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Target TPS", shooterTargetTPS);
        telemetry.addData("ShooterActive", shooterActive);
        telemetry.addData("IndexEncoder:", IndexValue);
        telemetry.addData("Heading (deg)", "%.1f", headingDeg);
<<<<<<< HEAD
        telemetry.addData("Magnetic switch state:", limitSwitchState);
        telemetry.addData("Index State:", indexActive);

// Ball detector with timed auto-index
// Start auto-indexing when a ball is detected and no other index mode is active
        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && indexActive != 1){
            ballCount = ballCount + 1;
=======
        telemetry.addData("AutoAim", autoAimActive);

        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && !IndexActive) {
>>>>>>> parent of 163ddd0 (k)
            autoIndexing = true;
            autoIndexStartTime = getRuntime();
            telemetry.addLine("The Ball Is Inside");
        }

// If we are in auto-index mode, run the indexer for 1 second
    /*    if (autoIndexing) { // If user starts another index mode, cancel auto-indexing

            if (!limitSwitchState){  // Keep indexer running forward
                if (ballCount % 2 != 0) {
                    //Index.setPower(-IndexPower);
                }
                else{
                    Index.setPower(stop);
                    autoIndexing = false;
                }
            }
            else if (limitSwitchState){
                    Index.setPower(stop);
                    autoIndexing = false;
            }

    }*/


        telemetry.update();

    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}