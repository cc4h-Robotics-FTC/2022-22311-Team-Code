package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;


@Autonomous
//Adds Code to Driver Stations
public abstract class HelloWorld extends OpMode {
    //Defines the class and if it is public or not
    @Override
    public void init(){
        //This code repeats and starts when init is pressed
        telemetry.addData("Hello FRANK", "Frank is SO COOL");
        //This is a test comma for my code so Yah
        /** Java Doc Comment YESSIR
         * I am so sus
         */
    }

    @Override
    public void loop(){
        //intentionally left blank

    }


}
