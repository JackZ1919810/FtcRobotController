package org.firstinspires.ftc.teamcode.TestFile;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@Disabled
@TeleOp(name = "MagSwitch_Test", group = "Test")
public class MagSwitch_Test extends OpMode {

    // Digital input for the REV Magnetic Limit Switch
    private DigitalChannel Mag_Switch;

    // Optional: a motor you want to control with the switch (e.g. indexer)
    private DcMotor testMotor;

    // For timing how fast the switch is seen by the code
    private boolean lastTriggeredState = false;
    private double lastChangeTime = 0;

    @Override
    public void init() {
        // Name "magSwitch" MUST match your configuration name in the Robot Controller app
        Mag_Switch = hardwareMap.get(DigitalChannel.class, "Mag_Switch");
        Mag_Switch.setMode(DigitalChannel.Mode.INPUT);

        // Optional motor (comment these two lines out if you don’t have/need a motor)
        lastChangeTime = getRuntime();

        telemetry.addLine("MagSwitch Test Initialized");
        telemetry.addLine("Move magnet near the sensor and watch the telemetry.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // REV magnetic switch is usually ACTIVE LOW:
        //   getState() == true  → NOT triggered
        //   getState() == false → TRIGGERED
        boolean rawState = Mag_Switch.getState();
        boolean triggered = !rawState;

        // Edge detection: did the state just change this loop?
        if (triggered != lastTriggeredState) {
            double now = getRuntime();
            double dt = now - lastChangeTime;

            telemetry.addLine("==== STATE CHANGED ====");
            telemetry.addData("Prev triggered", lastTriggeredState);
            telemetry.addData("Now triggered", triggered);
            telemetry.addData("Time since last change (s)", "%.3f", dt);

            lastTriggeredState = triggered;
            lastChangeTime = now;
        }

        // Continuous telemetry
        telemetry.addData("Raw getState()", rawState);
        telemetry.addData("Triggered (magnet present?)", triggered);
        telemetry.addData("Time since last change (s)", "%.3f", getRuntime() - lastChangeTime);
        telemetry.update();
    }
}
