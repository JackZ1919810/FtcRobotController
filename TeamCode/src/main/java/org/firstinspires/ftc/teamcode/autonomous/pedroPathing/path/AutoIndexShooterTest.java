package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
@Disabled
@Autonomous
public class AutoIndexShooterTest extends LinearOpMode {

    private DcMotor shooter1, shooter2;
    private DcMotor index;

    // pid constants
    private double kP = 0.000375;
    private double kI = 0.0;
    private double kD = 0.0;

    private double shooterTargetTPS = 2200;

    // pid state
    private int lastShooterPos = 0;
    private double lastTime = 0;
    private double shooterIntegral = 0;
    private double lastError = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        shooter1 = hardwareMap.get(DcMotor.class, "shooter_left");
        shooter2 = hardwareMap.get(DcMotor.class, "shooter_right");
        index  = hardwareMap.get(DcMotor.class, "Index");

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

        telemetry.addLine("READY TO SHOOT");
        telemetry.update();

        waitForStart();

        //start shooter
        double startTime = getRuntime();

        while (opModeIsActive()) {

            // shooter pid keeps running (target isnt ever reached but apparently this is the right speed? dont asking questions)
            double shooterTPS = getShooterSpeedTPS();
            double shooterPower = shooterPID(shooterTPS);

            shooter1.setPower(shooterPower);
            shooter2.setPower(shooterPower);

            telemetry.addData("TPS", shooterTPS);
            telemetry.addData("Target", shooterTargetTPS);
            telemetry.addData("ShooterPower", shooterPower);
            telemetry.update();

            // after 2 seconds run indexer
            if (getRuntime() - startTime > 2.0) {
                index.setPower(-0.4);  // continuous feed
            }

            // this auto just keeps feeding till time runs out
            idle();
        }
    }


    // tps calculation (again, dont ask questions)
    private double getShooterSpeedTPS() {
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

    // shooter pid loop (mind my business)
    private double shooterPID(double measuredTPS) {

        double error = shooterTargetTPS - measuredTPS;
        double dt = getRuntime() - lastTime;

        shooterIntegral += error * dt;
        double derivative = (error - lastError) / dt;

        double output = kP * error + kI * shooterIntegral + kD * derivative;

        lastError = error;

        return -Range.clip(output, 0, 1);
    }
}