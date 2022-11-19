package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public abstract class IfOpmode extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y < 0) ;
        telemetry.addData("Left Joystick Y", "Is Negative");
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
    }




}
