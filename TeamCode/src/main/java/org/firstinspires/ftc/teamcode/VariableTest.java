package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
@Disabled
@TeleOp
public class VariableTest extends OpMode {
    @Override
    public void init(){
        int teamNumber = 22310;
        double speed = 0.7;
        boolean driving = false;
        telemetry.addData("team number is", teamNumber);
        telemetry.addData("the speed is", speed);
        telemetry.addData("Is the robot driving?", driving);

    }
    @Override
    public void loop(){

    }

}
