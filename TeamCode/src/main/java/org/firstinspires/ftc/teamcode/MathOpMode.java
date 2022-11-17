package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public abstract class MathOpMode extends OpMode {
    @Override
    public void init(){

    }
    @Override
    public void loop(){
        double SpeedForward = -gamepad1.left_stick_y/2;
        double LeftStickX = -gamepad1.left_stick_x/2;
        Boolean ButtonB = gamepad1.b;
        double DifferenceSticks = SpeedForward - LeftStickX;
        double LeftTrigger = gamepad1.left_trigger;
        double RightTrigger = gamepad1.right_trigger;
        double DifferentTrig = LeftTrigger - RightTrigger;
        telemetry.addData("ButtonB's", ButtonB);
        telemetry.addData("Difference of Triggers", DifferentTrig);
        telemetry.addData("DifferenceOfJoysticks", DifferenceSticks);
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Left Stick X", LeftStickX);
        telemetry.addData("speed Forward", SpeedForward);
    }
}
