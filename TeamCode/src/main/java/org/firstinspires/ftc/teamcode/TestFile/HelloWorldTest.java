package org.firstinspires.ftc.teamcode.TestFile;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Disabled
@Autonomous
public class HelloWorldTest extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello", "Jack");
    }
    @Override
    public void loop(){

    }
}

