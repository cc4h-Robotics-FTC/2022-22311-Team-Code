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

            double distanceOfSensor = distanceSensor.getDistance(DistanceUnit.CM);

            while(distanceOfSensor > 2) {
                driveTrains.setPower(0,-0.2,0);
                distanceOfSensor = distanceSensor.getDistance(DistanceUnit.CM);
                telemetry.addData("distance > ", "%4.2f cm", distanceOfSensor);
                telemetry.update();
            }

            driveTrains.setPower(0,0,0);


            colorSensor.enableLed(true);
            sleep(250);
            telemetry.addData("Red > ", "%d", colorSensor.red());
            telemetry.addData("Blue > ", "%d", colorSensor.blue());
            telemetry.addData("Green > ", "%d", colorSensor.green());



            telemetry.update();
            colorSensor.enableLed(false);
        }


    }
}
