package org.firstinspires.ftc.teamcode.TestFile;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "TraceTagTest", group = "Test")
public class TestLimelightTracing extends OpMode {

    // Simple tank/mecanum style: we just use them for turning in place
    private DcMotor FLwheel, FRwheel, BLwheel, BRwheel;

    private Limelight3A limelight;

    // Tuning constants
    private static final double kP_TURN       = 0.02;  // proportional gain for turning
    private static final double MAX_TURN_PWR  = 0.5;   // max magnitude of turn power
    private static final double ANGLE_TOL_DEG = 2.0;   // within this many degrees → stop turning

    @Override
    public void init() {
        // Map drive motors (names must match your RC config)
        FLwheel = hardwareMap.get(DcMotor.class, "FLwheel");
        BLwheel = hardwareMap.get(DcMotor.class, "BLwheel");
        FRwheel = hardwareMap.get(DcMotor.class, "FRwheel");
        BRwheel = hardwareMap.get(DcMotor.class, "BRwheel");

        // Reverse one side so positive power drives forward
        FRwheel.setDirection(DcMotor.Direction.REVERSE);
        BRwheel.setDirection(DcMotor.Direction.REVERSE);

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);     // fast updates
        limelight.pipelineSwitch(0);      // use pipeline 0 (configure as AprilTag)
        limelight.start();

        telemetry.addLine("TraceTag20_24_Closest initialized.");
        telemetry.update();
    }

    @Override
    public void loop() {

        double turnPower = 0.0;
        boolean tagSeen = false;
        int trackedId = -1;
        double usedAngleDeg = 0.0;
        double usedDistance = 0.0;

        // Get latest result from Limelight
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {

                LLResultTypes.FiducialResult closestTag = null;
                double closestZ = Double.MAX_VALUE;  // forward distance in meters

                // 1) Find the closest tag among IDs 20 and 24
                for (LLResultTypes.FiducialResult fid : fiducials) {
                    int id = fid.getFiducialId();
                    if (id == 20 || id == 24) {
                        Pose3D poseCamSpace = fid.getTargetPoseCameraSpace();
                        if (poseCamSpace != null) {
                            double z = poseCamSpace.getPosition().z; // forward (meters)
                            if (z < closestZ) {
                                closestZ = z;
                                closestTag = fid;
                            }
                        }
                    }
                }

                // 2) If we found at least one of 20/24, track the closest one
                if (closestTag != null) {
                    tagSeen = true;
                    trackedId = closestTag.getFiducialId();

                    Pose3D poseCamSpace = closestTag.getTargetPoseCameraSpace();
                    double x = poseCamSpace.getPosition().x; // left/right (meters)
                    double z = poseCamSpace.getPosition().z; // forward (meters)

                    // Horizontal angle from camera to tag (radians → degrees)
                    // atan2(x, z): if x>0 → tag to the left/right depending on coord system.
                    double angleRad = Math.atan2(x, z);
                    double txDeg = Math.toDegrees(angleRad);

                    usedAngleDeg = txDeg;
                    usedDistance = Math.hypot(x, z);

                    // Simple proportional turn controller
                    turnPower = -txDeg * kP_TURN;

                    // Clamp
                    if (turnPower >  MAX_TURN_PWR) turnPower =  MAX_TURN_PWR;
                    if (turnPower < -MAX_TURN_PWR) turnPower = -MAX_TURN_PWR;

                    // Deadband near center
                    if (Math.abs(txDeg) < ANGLE_TOL_DEG) {
                        turnPower = 0.0;
                    }
                }
            }
        }

        // Apply turn power to drive (turn in place)
        double leftPower  = -turnPower;
        double rightPower =  turnPower;

        FLwheel.setPower(leftPower);
        BLwheel.setPower(leftPower);
        FRwheel.setPower(rightPower);
        BRwheel.setPower(rightPower);

        // Telemetry
        telemetry.addData("Tag 20/24 seen", tagSeen);
        telemetry.addData("Tracked ID", trackedId);
        telemetry.addData("Angle to tag (deg)", "%.2f", usedAngleDeg);
        telemetry.addData("Distance to tag (m)", "%.2f", usedDistance);
        telemetry.addData("Turn power", "%.2f", turnPower);
        telemetry.update();
    }
}
