package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="Robot: 22133AutonomousEncoder")
public class TeamRobotAutoEncoder extends LinearOpMode {

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
            //Drive to cone!!
            autoDrive(0,-0.2,0, 1200);
            // Read the cone
            colorSensor.enableLed(true);
            sleep(250);
            int ColorInteger = readColor();

            telemetry.addData("Go to Pos ", "%d",ColorInteger);
            telemetry.update();

            sleep(1000);
            //push cone out of the way
            autoDrive(0,-0.2,0, 800);
            autoDrive(0,0.2,0, 300);







            if(ColorInteger == 1){
                PositionOne();
            }
            else if(ColorInteger == 2){
                PositionTwo();
            }
            else {
                PositionThree();
            }
//            telemetry.addData("Red > ", "%d", colorSensor.red());
//            telemetry.addData("Blue > ", "%d", colorSensor.blue());
//            telemetry.addData("Green > ", "%d", colorSensor.green());



            telemetry.update();
            colorSensor.enableLed(false);
            break;
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


        autoDrive(0.2, 0, 0, 1250);

    }
    public void PositionOne(){
        autoDrive(-0.2, 0, 0, 1300);

    }
    public void PositionTwo() {

    }



    private void autoDrive(double axial, double lateral, double yaw, int pos){
        driveTrains.init_encoders();

        driveTrains.setPower(axial,lateral,yaw);

        while (Math.abs(driveTrains.getPosition()) < pos ) {
            telemetry.addData("Go to Pos ", "%d",driveTrains.getPosition());
            telemetry.update();
        }
        driveTrains.setPower(0,0,0);
    }
}

