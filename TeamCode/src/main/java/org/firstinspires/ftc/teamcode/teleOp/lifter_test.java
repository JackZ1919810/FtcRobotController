package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Servo Detector Test")
public class lifter_test extends LinearOpMode {

    private CRServo myServo;
    private CRServo myServo1;
    private CRServo myServo2;
    private CRServo myServo3;
    private CRServo myServo4;
    private CRServo myServo5;
    private CRServo myServo6;
    private CRServo myServo7;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Checking for servos...");
        telemetry.update();

        // We use a try-catch block to "detect" the servos without crashing if they are missing
        try {
            myServo = hardwareMap.get(CRServo.class, "myServo");
            myServo1 = hardwareMap.get(CRServo.class, "myServo1");
            myServo2 = hardwareMap.get(CRServo.class, "myServo2");
            myServo3 = hardwareMap.get(CRServo.class, "myServo3");
            myServo4 = hardwareMap.get(CRServo.class, "myServo4");
            myServo5 = hardwareMap.get(CRServo.class, "myServo5");
            myServo6 = hardwareMap.get(CRServo.class, "myServo6");
            myServo7 = hardwareMap.get(CRServo.class, "myServo7");


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
                    myServo.setPower(-1);
                    myServo1.setPower(-1);
                    myServo2.setPower(1);
                    myServo3.setPower(1);
                    myServo4.setPower(-1);
                    myServo5.setPower(1);
                    myServo6.setPower(1);
                    myServo7.setPower(-1);

                }
                else if (gamepad1.b) {
                    // Move DOWN
                    myServo.setPower(1);
                    myServo1.setPower(1);
                    myServo2.setPower(-1);
                    myServo3.setPower(-1);
                    myServo4.setPower(1);
                    myServo5.setPower(-1);
                    myServo6.setPower(-1);
                    myServo7.setPower(1);

                }
                else {
                    // STOP
                    myServo.setPower(0);
                    myServo1.setPower(0);
                    myServo2.setPower(0);
                    myServo3.setPower(0);
                    myServo4.setPower(0);
                    myServo5.setPower(0);
                    myServo6.setPower(0);
                    myServo7.setPower(0);
                    
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