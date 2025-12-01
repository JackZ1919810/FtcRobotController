package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@Disabled
@TeleOp(name = "HolonomicDrive_DirectionTest", group = "Debug")
public class HolonomicDrive_DirectionTest extends LinearOpMode {
    // TODO: change these to match your Robot Configuration
    private static final String LF_NAME = "FLwheel";
    private static final String RF_NAME = "FRwheel";
    private static final String LR_NAME = "BLwheel";
    private static final String RR_NAME = "BRwheel";

    private DcMotor lf, rf, lr, rr;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. Map motors
        lf = hardwareMap.get(DcMotor.class, LF_NAME);
        rf = hardwareMap.get(DcMotor.class, RF_NAME);
        lr = hardwareMap.get(DcMotor.class, LR_NAME);
        rr = hardwareMap.get(DcMotor.class, RR_NAME);

        // 2. Set directions (START with a common guess, then adjust based on behavior)
        // For many holonomic / mecanum / X-drive builds, this is a good starting point:
        lf.setDirection(DcMotorSimple.Direction.REVERSE);
        lr.setDirection(DcMotorSimple.Direction.REVERSE);
        rf.setDirection(DcMotorSimple.Direction.FORWARD);
        rr.setDirection(DcMotorSimple.Direction.FORWARD);

        // Optional but recommended
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("HolonomicDirectionTest");
        telemetry.addLine("A/B/X/Y: test single motors");
        telemetry.addLine("Dpad Up: pure forward");
        telemetry.addLine("Dpad Down: pure backward");
        telemetry.addLine("Dpad Left: strafe left");
        telemetry.addLine("Dpad Right: strafe right");
        telemetry.addLine("Left stick: drive (forward/back + strafe)");
        telemetry.addLine("Right stick X: rotate");
        telemetry.addLine("Press START on driver station to begin.");
        telemetry.update();

        waitForStart();

        double testPower = 0.4; // power for button tests

        while (opModeIsActive()) {
            // --- Single motor tests (hold button to run motor) ---
            double lfPower = 0.0;
            double rfPower = 0.0;
            double lrPower = 0.0;
            double rrPower = 0.0;

            // A: left front
            if (gamepad1.a) {
                lfPower = testPower;
                telemetry.addLine("Testing LEFT FRONT motor (A)");
            }

            // B: right front
            if (gamepad1.b) {
                rfPower = testPower;
                telemetry.addLine("Testing RIGHT FRONT motor (B)");
            }

            // X: left rear
            if (gamepad1.x) {
                lrPower = testPower;
                telemetry.addLine("Testing LEFT REAR motor (X)");
            }

            // Y: right rear
            if (gamepad1.y) {
                rrPower = testPower;
                telemetry.addLine("Testing RIGHT REAR motor (Y)");
            }

            // If any of A/B/X/Y are pressed, run ONLY that motor and skip the rest
            boolean singleMotorTest =
                    gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y;

            if (singleMotorTest) {
                lf.setPower(lfPower);
                rf.setPower(rfPower);
                lr.setPower(lrPower);
                rr.setPower(rrPower);

                telemetry.addData("LF", lfPower);
                telemetry.addData("RF", rfPower);
                telemetry.addData("LR", lrPower);
                telemetry.addData("RR", rrPower);
                telemetry.update();
                continue;
            }

            // --- Pattern tests: forward/back/strafe using DPAD ---
            double p = testPower;

            if (gamepad1.dpad_up) {
                // PURE FORWARD
                lfPower = p;
                rfPower = p;
                lrPower = p;
                rrPower = p;
                telemetry.addLine("Pattern: PURE FORWARD (dpad_up)");
            } else if (gamepad1.dpad_down) {
                // PURE BACKWARD
                lfPower = -p;
                rfPower = -p;
                lrPower = -p;
                rrPower = -p;
                telemetry.addLine("Pattern: PURE BACKWARD (dpad_down)");
            } else if (gamepad1.dpad_left) {
                // PURE STRAFE LEFT (mecanum/X-drive style)
                // Adjust signs if your robot doesn't strafe correctly.
                lfPower = -p;
                rfPower = +p;
                lrPower = +p;
                rrPower = -p;
                telemetry.addLine("Pattern: STRAFE LEFT (dpad_left)");
            } else if (gamepad1.dpad_right) {
                // PURE STRAFE RIGHT
                lfPower = +p;
                rfPower = -p;
                lrPower = -p;
                rrPower = +p;
                telemetry.addLine("Pattern: STRAFE RIGHT (dpad_right)");
            } else {
                // --- Normal holonomic drive with sticks ---
                // LEFT STICK Y: forward/back (negated because up is -1)
                // LEFT STICK X: strafe
                // RIGHT STICK X: rotate
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double rotate = gamepad1.right_stick_x;

                // Standard mecanum-style mixing (also works as X-drive base)
                lfPower = forward + strafe + rotate;
                rfPower = forward - strafe - rotate;
                lrPower = forward - strafe + rotate;
                rrPower = forward + strafe - rotate;

                // Normalize if needed so no value exceeds 1.0
                double max = Math.max(1.0, Math.max(
                        Math.abs(lfPower),
                        Math.max(Math.abs(rfPower),
                                Math.max(Math.abs(lrPower), Math.abs(rrPower)))
                ));

                lfPower /= max;
                rfPower /= max;
                lrPower /= max;
                rrPower /= max;

                telemetry.addLine("Mode: joystick holonomic drive");
                telemetry.addData("Forward", "%.2f", forward);
                telemetry.addData("Strafe", "%.2f", strafe);
                telemetry.addData("Rotate", "%.2f", rotate);
            }

            // Apply powers
            lf.setPower(lfPower);
            rf.setPower(rfPower);
            lr.setPower(lrPower);
            rr.setPower(rrPower);

            telemetry.addData("LF power", "%.2f", lfPower);
            telemetry.addData("RF power", "%.2f", rfPower);
            telemetry.addData("LR power", "%.2f", lrPower);
            telemetry.addData("RR power", "%.2f", rrPower);
            telemetry.update();
        }

        // Stop all motors when OpMode ends
        lf.setPower(0);
        rf.setPower(0);
        lr.setPower(0);
        rr.setPower(0);
    }
}


