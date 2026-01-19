package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Servo Detector Test")
public class lifter_test extends LinearOpMode {

    private CRServo myServo;
    private CRServo myServo1;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Checking for servos...");
        telemetry.update();

        // We use a try-catch block to "detect" the servos without crashing if they are missing
        try {
            myServo = hardwareMap.get(CRServo.class, "myServo");
            myServo1 = hardwareMap.get(CRServo.class, "myServo1");

            // If the code reaches here, it found them!
            telemetry.addData("Servos Detected", "YES");
            telemetry.addData("Status", "Ready to start");
            telemetry.update();

        } catch (Exception e) {
            // If the names "myServo" or "myServo1" are missing from the config
            telemetry.addData("Servos Detected", "NO (Check Config)");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            // We only run the movement code if the servos were actually found (not null)
            if (myServo != null && myServo1 != null) {

                if (gamepad1.a) {
                    // Move UP
                    myServo.setPower(1);
                    myServo1.setPower(-1);
                }
                else if (gamepad1.b) {
                    // Move DOWN
                    myServo.setPower(-1);
                    myServo1.setPower(1); // Fixed: changed myServo to myServo1
                }
                else {
                    // STOP
                    myServo.setPower(0);
                    myServo1.setPower(0); // Fixed: changed myServo to myServo1
                }

                // Keep showing the status while running
                telemetry.addData("Servos Detected", "YES");
                telemetry.addData("Servo Power", myServo.getPower());
            } else {
                telemetry.addData("Servos Detected", "NO");
            }

            telemetry.update();
        }
    }
}