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
        colorSensor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");

        waitForStart();
        while (opModeIsActive()) {

            double distanceOfSensor = distanceSensor.getDistance(DistanceUnit.MM);

            driveTrains.setPower(0,-0.2,0);
            while(distanceOfSensor > 100) {
                telemetry.addData("distance > ", "%4.2f mm", distanceOfSensor);
                distanceOfSensor = distanceSensor.getDistance(DistanceUnit.MM);
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
