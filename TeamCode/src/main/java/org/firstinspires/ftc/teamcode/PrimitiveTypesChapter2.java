package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public abstract class PrimitiveTypesChapter2 extends OpMode {
    @Override
    public void init() {
        int teamNumber = 16072;
        boolean touchSensorPressed = true;
        double motorspeed = 0.5;
        telemetry.addData("TeamNumber","teamnumber" );
        telemetry.addData("TouchSensorPressed", "touchSensorPressed");
        telemetry.addData("MotorSpeed", "motorspeed");

    }
    @Override
    public void loop() {

    }

}
