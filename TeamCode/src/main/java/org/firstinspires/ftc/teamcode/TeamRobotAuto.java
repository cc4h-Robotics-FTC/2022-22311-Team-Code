package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Autonomous(name="Robot: 22133Autonomous")
public class TeamRobotAuto extends LinearOpMode {

    private ColorSensor colorSensor = null;
    private DistanceSensor distanceSensor = null;
    DriveTrains driveTrains = null;
    @Override
    public void runOpMode() throws InterruptedException {

        driveTrains = new DriveTrains(this);
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color");

        waitForStart();
        while (opModeIsActive()) {
            int LoopCount = 0;
            double distanceOfSensor = distanceSensor.getDistance(DistanceUnit.CM);

            while(distanceOfSensor > 2) {
                LoopCount++;
                driveTrains.setPower(0, -0.2, 0);
                distanceOfSensor = distanceSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("distance > ", "%4.2f cm", distanceOfSensor);
                telemetry.update();

            }

            telemetry.addData("LoopsCount", LoopCount );

            driveTrains.setPower(0,0,0);


            colorSensor.enableLed(true);
            sleep(250);
            int ColorInteger = readColor();
            if(ColorInteger == 1){
                telemetry.addData("Pos 1", 1);
                PositionOne();
            }
            else if(ColorInteger == 2){
                telemetry.addData("Pos 2", 2);
                PositionTwo();
            }
            else {
                telemetry.addData("Pos 3", 3);
                PositionThree();
            }
            telemetry.addData("Red > ", "%d", colorSensor.red());
            telemetry.addData("Blue > ", "%d", colorSensor.blue());
            telemetry.addData("Green > ", "%d", colorSensor.green());



            telemetry.update();
            colorSensor.enableLed(false);
        }


    }
    public int readColor() {
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();


        int largerNumber = Math.max(red, blue);
        int LargestNumberChoice = Math.max(green, largerNumber);

        if (LargestNumberChoice == red) {
            return 1;
        }

        else if (LargestNumberChoice == blue){
            return 2;
        }

        else if (LargestNumberChoice == green){
            return 3;
        }


        return 0;
    }
    public void PositionThree(){
        driveTrains.setPower(-0.2,0,0 );
    }
    public void PositionOne(){
        driveTrains.setPower(0.2, 0, 0);
    }
    public void PositionTwo(){
        driveTrains.setPower(0, 0, 0);
    }



}

