package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "TeleOp ", group = "TeleOp")
public class OmniDriveTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private Servo Servo;

    private static final double DEADZONE = 0.06;
    private static final double TURN_SCALE = 0.9;

    // Shooter PID constants (start with these and tune)
    private double kP = 0.0004;
    private double kI = 0.0000;
    private double kD = 0.0000;

    // Target shooter speed in ticks per second (tune this!)
    private double shooterTargetTPS = 2000.0;

    // Shooter PID state
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;

    private double IntakePower = 1;
    private double IndexPower = 0.4;
    private double stop = 0;

    private boolean reverseControls = false;
    private boolean lastAState = false;
    private double ServoPosition1 = 0;
    private double Servoposition2 = 0.5;

    private boolean shoot_far = true;

    @Override
    public void init() {
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");
        Servo = hardwareMap.get(Servo.class, "Servo1");

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        IntakeMotor = hardwareMap.get(DcMotor.class, "intake");
        Index = hardwareMap.get(DcMotor.class, "Index");

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        IntakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Enable encoders for shooter PID
        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lastShooterPos = shooter1.getCurrentPosition();
        lastTime = getRuntime();
    }

    @Override
    public void loop() {
        // ========== DRIVE CONTROLS ==========
        boolean currentAState = gamepad1.a;
        if (currentAState && !lastAState) {
            reverseControls = !reverseControls;
        }
        lastAState = currentAState;

        //detect pressing time of the distance changing button
        if (gamepad2.dpad_up){
            shoot_far = true;
        }
        else if (gamepad2.dpad_down){
            shoot_far = false;
        }

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

        // ========== GAMEPAD LOGIC ==========
        boolean shooterActive_short = (gamepad2.right_trigger > 0.1 && shoot_far == false);
        boolean shooterActive_long = (gamepad2.right_trigger > 0.1 && shoot_far == true);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean IndexActive = (gamepad2.right_bumper);
        boolean BallsOut = (gamepad2.left_bumper);
        boolean ServoActive = (gamepad1.right_trigger > 0.1);


       /* if (gamepad2.dpad_up) {
            shooterTargetTPS += 100; // increase target speed
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS -= 100; // decrease target speed
        }
        shooterTargetTPS = Range.clip(shooterTargetTPS, 0, 5000); // clamp reasonable range
        */

        // ========== BALLS OUT (REVERSE ALL FEED) ==========
        if (BallsOut) {
            Index.setPower(IndexPower);
            IntakeMotor.setPower(IntakePower);
        } else {
            Index.setPower(stop);
            IntakeMotor.setPower(stop);
        }

        // ========== SERVO ==========
        if (ServoActive) {
            Servo.setPosition(Servoposition2);
        } else {
            Servo.setPosition(ServoPosition1);
        }

        // ========== SHOOTER PID CONTROL ==========
        // Measure shooter speed
        int currentPos = shooter1.getCurrentPosition();
        double currentTime = getRuntime();
        double dt = currentTime - lastTime;
        double shooterTPS = 0;

        if (dt > 0) {
            shooterTPS = (currentPos - lastShooterPos) / dt; // ticks per second
        }

        //short distance shooter

        if (shooterActive_short) {

            shooterTargetTPS = 2000;
            // PID on absolute speed, so direction sign doesn't matter
            double currentSpeed = Math.abs(shooterTPS);
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

            // Shooter power command (negative because you used -0.67 before)
            double shooterPower = -Range.clip(output, 0, 1);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // While shooter is active, you previously fed Index backwards
            Index.setPower(-IndexPower);

            lastError = error;

        } else {
            // Shooter off, reset PID state
            shooter1.setPower(0);
            shooter2.setPower(0);
            shooterIntegral = 0;
            lastError = 0;
        }

        //long range shooter
        if (shooterActive_long) {

            shooterTargetTPS = 2200;
            // PID on absolute speed so direction sign doesn't matter
            double currentSpeed = Math.abs(shooterTPS);
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

            // Shooter power command (negative because you used -0.67 before)
            double shooterPower = -Range.clip(output, 0, 1);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            // While shooter is active, you previously fed Index backwards
            Index.setPower(-IndexPower);

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

        // ========== INDEX MANUAL FEED ==========
        if (IndexActive) {
            Index.setPower(-IndexPower);
        } else if (!shooterActive_short && !shooterActive_short && !BallsOut && !intakeActive) {
            // Only stop here if nothing else is trying to move the Index
            Index.setPower(stop);
        }

        // ========== INTAKE ==========
        if (intakeActive) {
            IntakeMotor.setPower(-IntakePower);
            Index.setPower(-IndexPower);
        } else if (!BallsOut) {
            IntakeMotor.setPower(stop);
        }

        // (Optional) telemetry to help tune PID
        telemetry.addData("Shooter TPS", shooterTPS);
        telemetry.addData("Target TPS", shooterTargetTPS);
        telemetry.addData("ShooterActive", shooterActive_long || shooterActive_short);
        telemetry.update();
    }

    private static double applyDeadzone(double v) {
        return (Math.abs(v) < DEADZONE) ? 0.0 : v;
    }
}
