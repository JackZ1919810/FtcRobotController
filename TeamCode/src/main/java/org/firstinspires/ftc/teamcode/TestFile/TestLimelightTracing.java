package org.firstinspires.ftc.teamcode.TestFile;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.TestFile.AprilTag;
import org.firstinspires.ftc.teamcode.Point;

import java.util.List;
@TeleOp
public class TestLimelightTracing extends OpMode{

        // Simple tank drive example — rename motors to match your config
        private DcMotor leftFront, leftBack, rightFront, rightBack;

        // Limelight
        private Limelight3A limelight;

        // Max turn power
        private static final double MAX_TURN_SPEED = 0.4;

        @Override
        public void init() {

            // --- Map your motors ---
            leftFront  = hardwareMap.get(DcMotor.class, "FLwheel");
            leftBack   = hardwareMap.get(DcMotor.class, "BLwheel");
            rightFront = hardwareMap.get(DcMotor.class, "FRwheel");
            rightBack  = hardwareMap.get(DcMotor.class, "BRwheel");

            // Reverse one side
            rightFront.setDirection(DcMotor.Direction.REVERSE);
            rightBack.setDirection(DcMotor.Direction.REVERSE);

            // --- Limelight ---
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);     // fast updates
            limelight.pipelineSwitch(0);
            limelight.start();

            telemetry.addLine("Initialized. Tracing AprilTag ID 20 using utility class.");
            telemetry.update();
        }

        @Override
        public void loop() {

            // By default, motors do nothing
            double turnPower = 0;
            boolean tag20Seen = false;

            // Read Limelight result
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                if (fiducials != null && !fiducials.isEmpty()) {

                    // ---- Get Tag ID using your utility function ----
                    int id = AprilTag.getTagID(fiducials);

                    if (id == 20) {
                        tag20Seen = true;


                        double tx = result.getTx();   // horizontal angle offset (degrees)

                        // ---- Use your utility class to calculate turning ----
                        // This calls: AprilTag.trackTag(motor, maxTurnSpeed, tx)
                        // But trackTag only drives ONE motor.
                        // So we use it to compute power for a "turn", then apply to both sides.

                        // Use a fake motor to extract the power? No — trackTag *sets* motor power.
                        // Instead, we REPLICATE how trackTag clamps power, then apply it to all motors:

                        double kP = 0.015;
                        turnPower = -tx * kP;
                        turnPower = Math.max(-MAX_TURN_SPEED, Math.min(MAX_TURN_SPEED, turnPower));

                        if (Math.abs(tx) < 3.0) {
                            turnPower = 0;
                        }

                        // (Optional) Read position using your utility class
                        // Pose3D pose = tag.getRobotPoseTargetSpace();
                        // Point pos = AprilTag.getPosition(pose);

                    }
                }
            }

            // ---- Apply turn power ----
            double leftPower = -turnPower;
            double rightPower = turnPower;

            leftFront.setPower(leftPower);
            leftBack.setPower(leftPower);
            rightFront.setPower(rightPower);
            rightBack.setPower(rightPower);

            // ---- Telemetry ----
            telemetry.addData("Tag 20 Visible", tag20Seen);
            telemetry.addData("Turn Power", turnPower);
            telemetry.update();
        }
}


