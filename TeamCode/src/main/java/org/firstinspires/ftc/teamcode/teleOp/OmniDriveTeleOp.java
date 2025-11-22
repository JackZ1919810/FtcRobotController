package org.firstinspires.ftc.teamcode.teleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Base64;

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;
    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;

    // Shooter PID constants (start with these and tune)
    private double kP = 0.0004;
    private double kI = 0.0000;
    private double kD = 0.0000;

    // Auto index (distance-based) state
    private boolean autoIndexing = false;
    private double autoIndexStartTime = 0;


    // Target shooter speed in ticks per second (tune this!)
    private double shooterTargetTPS = 2600.0;

    // Shooter PID state
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

        // setMode(DcMotor.RunMode.RUN_USING_ENCODER)

        // Enable encoders for shooter PID
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Index.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Index.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


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

    }

    @Override
    public void loop() {

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

        if (reverseControls) {
            lx = -lx;
            ly = -ly;
            rx = -rx;
        }

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
        boolean IndexActive = (gamepad2.right_bumper);
        boolean BallsOut = (gamepad2.left_bumper);
        // distance variable
        double DistanceOn = (Distance.getDistance(DistanceUnit.CM));

        telemetry.addData("Distance is ", Distance.getDistance(DistanceUnit.CM) );

        //  adjust shooter target speed with dpad
        if (gamepad2.dpad_up) {
            shooterTargetTPS = 2600; // Far target
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 2200; // close target
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
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = 0;

        if (dt > 0) {
            shooterTPS = (currentPos - lastShooterPos) / dt; // ticks per second
        }

        if (shooterActive) {
            // PID on absolute speed so direction sign doesn't matter
            double currentSpeed = Math.abs(shooterTPS);
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

            // Shooter power command
            double shooterPower = -Range.clip(output, 0, 1);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);


            lastError = error;
        } else {
            // Shooter off, reset PID state
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterIntegral = 0;
            lastError = 0;
        }

        // Update last encoder/time for next loop
        lastShooterPos = currentPos;
        lastTime = currentTime;

        // Index
        if (IndexActive) {
            Index.setPower(-IndexPower);
        } else if (!shooterActive && !BallsOut && !intakeActive) {
            // Only stop here if nothing else is trying to move Index
            Index.setPower(stop);
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

        // Ball detector with timed auto-index
        // Start auto-indexing when a ball is detected and no other index mode is active
        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && !IndexActive) {
            autoIndexing = true;
            autoIndexStartTime = getRuntime();
            telemetry.addLine("The Ball Is Inside");
        }

        // If we are in auto-index mode, run the indexer for 1 second
        if (autoIndexing) {
            // If user starts another index mode, cancel auto-indexing
            if (BallsOut || IndexActive) {
                autoIndexing = false;
            } else {
                // Keep indexer running forward
                Index.setPower(-IndexPower);

                // Stop after 1 second
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
