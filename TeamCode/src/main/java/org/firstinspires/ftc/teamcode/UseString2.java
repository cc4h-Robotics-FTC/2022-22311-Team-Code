package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public abstract class UseString2 extends OpMode {
    @Override
    public void init() {
        String MyName = "Franklin Hsu the Great, Powerful, and Mighty";
        int Grade = 100;
        telemetry.addData("Hello", MyName);
        telemetry.addData("Sussy Grade", Grade);
    }




//    @Override
//    public void loop() {
//    int X = 17;
//    // only X visible here
//    {
//        int Y = 19;
//    }
//    }
//    //X and Y are visible here
//        // {
//                //Only X visible here
//            }
//
//        }



}