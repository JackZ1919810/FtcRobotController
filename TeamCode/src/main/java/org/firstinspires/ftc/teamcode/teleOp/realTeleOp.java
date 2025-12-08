package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.Range;


import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;

// Limelight imports
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp
public class realTeleOp extends OpMode {

    private DcMotor FRwheel, FLwheel, BRwheel, BLwheel;
    private DcMotor shooter1, shooter2;
    private DcMotor IntakeMotor;
    private DcMotor Index;
    private DistanceSensor Distance;

    // Limelight
    private Limelight3A limelight;

    //Auto align mode state
    private boolean autoAlignEnabled = false;
    private boolean lastAlignButton = false;

    //P Control for LL alignment
    private static final double kP_LL_Turn = 0.01;
    private static final double Max_LL_Turn = 0.5;
    private static final double LL_Angle_Range_DEG = 2.0;

    private DigitalChannel Mag_Switch;

    private boolean limitSwitchState;
    private static final double DEADZONE = 0.09;
    private static final double TURN_SCALE = 0.7;

    // Shooter PID constants (start with these and tune)
    private double kP = 0.003;
    private double kI = 0.0015;
    private double kD = 0.000;

    // Auto index (distance-based) state
    private boolean autoIndexing = false;
    private int indexActive = 0;

    private int ballCount = 0;
    private double autoIndexStartTime = 0;


    // Target shooter speed in ticks per second (tune this!)
    private double shooterTargetTPS = 1000.0;

    // Shooter PID state
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;
    private double startTime = 0;
    private int shooterLoopTimes = 0;

    private double IntakePower =  1;
    private double IndexPower = 0.24;
    private double stop = 0;

    private boolean reverseControls = false;
    private boolean lastAState = false;

    private IMU imu;

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

        //Limelight init
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

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

        //  GAMEPAD buttons
        boolean shooterActive = (gamepad2.right_trigger > 0.1);
        boolean intakeActive = (gamepad2.left_trigger > 0.1);
        boolean BallsOut = (gamepad2.left_bumper);

        //Index Gamepad logic
        if(gamepad2.a){
            indexActive = 1;
        }

        //Auto_align mode button
        boolean alignButton = gamepad1.right_bumper;

        if (alignButton && !lastAlignButton){
            autoAlignEnabled = !autoAlignEnabled;
        }
        lastAlignButton = alignButton;

        double DistanceOn = (Distance.getDistance(DistanceUnit.CM));

        telemetry.addData("Distance is ", Distance.getDistance(DistanceUnit.CM) );

//  adjust shooter target speed with dpad
        if (gamepad2.dpad_up) {
            shooterTargetTPS = 1150; // Far target
        } else if (gamepad2.dpad_down) {
            shooterTargetTPS = 1000; // close target
        }
        shooterTargetTPS = Range.clip(shooterTargetTPS, 0, 5000); // clamp reasonable range


        double lx = applyDeadzone(Math.pow(gamepad1.left_stick_x, 3));
        double ly = applyDeadzone(Math.pow(-gamepad1.left_stick_y, 3));
        double rxManual = applyDeadzone(Math.pow(gamepad1.right_stick_x, 3)) * TURN_SCALE;

        if (reverseControls) {
            lx = -lx;
            ly = -ly;
            rxManual = -rxManual;
        }

        double rx;
        boolean tagSeen = false;
        int trackedId = -1;
        double usedAngleDeg = 0.0;

        rx = rxManual;

        if (autoAlignEnabled){

            LLResult result= limelight.getLatestResult();

            if (result != null && result.isValid()){

                List<FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()){
                    FiducialResult closestTag = null;
                    double closestZ = Double.MAX_VALUE;

                    for (FiducialResult fid : fiducials) {
                        int id = fid.getFiducialId();
                        if (id == 20 || id == 24) {
                            Pose3D poseCamSpace = fid.getTargetPoseCameraSpace();
                            if (poseCamSpace != null) {
                                double z = poseCamSpace.getPosition().z;
                                if (z < closestZ) {
                                    closestZ  = z;
                                    closestTag = fid;
                                }
                            }
                        }
                    }

                    if (closestTag != null){
                        tagSeen = true;
                        trackedId = closestTag.getFiducialId();

                        Pose3D poseCamSpace = closestTag.getTargetPoseCameraSpace();
                        double x = poseCamSpace.getPosition().x;
                        double z = poseCamSpace.getPosition().z;

                        double angleRad=Math.atan2(x, z);
                        double txDeg = Math.toDegrees(angleRad);
                        usedAngleDeg = txDeg;

                        double turnCmd = txDeg * kP_LL_Turn;

                        if (turnCmd > Max_LL_Turn) turnCmd = Max_LL_Turn;
                        if (turnCmd < -Max_LL_Turn) turnCmd = -Max_LL_Turn;

                        if (Math.abs(txDeg) < LL_Angle_Range_DEG){
                            turnCmd = 0.0;
                        }

                        telemetry.addData("turnCMD", turnCmd);

                        rx = turnCmd;
                    }
                }
            }
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

        if (shooterActive && !BallsOut) {

            if (shooterLoopTimes == 0){
                startTime = getRuntime();
            }

            indexActive = 0;
            double currentSpeed = Math.abs(shooterTPS); // PID on absolute speed so direction sign doesn't matter
            double error = shooterTargetTPS - currentSpeed;

            shooterIntegral += error * dt;
            double derivative = (dt > 0) ? (error - lastError) / dt : 0;

            double output = kP * error + kI * shooterIntegral + kD * derivative;

// Shooter power command
            double shooterPower = -Range.clip(-output, -1, 0);

            shooter1.setPower(-shooterPower);
            shooter2.setPower(-shooterPower);

            telemetry.addData("shooter run time", getRuntime()-startTime);

            if (getRuntime()-startTime >= 1){
                IndexPower = 0.5;
                Index.setPower(-IndexPower);
            }

            lastError = error;
            shooterLoopTimes = shooterLoopTimes + 1;
        }
        else if (!shooterActive){
// Shooter off, reset PID state
            IndexPower = 0.24;
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
                if (gamepad2.b){
                    Index.setPower(stop);
                    indexActive = 0;
                }
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
        telemetry.addData("Magnetic switch state:", limitSwitchState);
        telemetry.addData("Index State:", indexActive);
        telemetry.addData("AutoAlign", autoAlignEnabled);
        telemetry.addData("Tag seen", tagSeen);
        telemetry.addData("Tracked ID", trackedId);
        telemetry.addData("LL angle (deg)", "%.2f", usedAngleDeg);
        telemetry.addData("rx (final)", "%.2f", rx);

        telemetry.update();


// Ball detector with timed auto-index
// Start auto-indexing when a ball is detected and no other index mode is active
        if (DistanceOn < 2.4 && !autoIndexing && !BallsOut && indexActive != 1){
            ballCount = ballCount + 1;
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