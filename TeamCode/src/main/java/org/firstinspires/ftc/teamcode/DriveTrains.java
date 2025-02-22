package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveTrains {
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private LinearOpMode opMode = null;

    public DriveTrains(LinearOpMode opMode) {
        this.opMode = opMode;

        leftFrontDrive = opMode.hardwareMap.get(DcMotor.class, "Left_Front");
        leftBackDrive = opMode.hardwareMap.get(DcMotor.class, "Left_Rear");
        rightFrontDrive = opMode.hardwareMap.get(DcMotor.class, "Right_Front");
        rightBackDrive = opMode.hardwareMap.get(DcMotor.class, "Right_Rear");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

    }

    public void setPower(double axial, double lateral, double yaw) {
        double max;
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;


        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

//            max = Math.max(max, Math.abs(ClawForwardPower));
//            max = Math.max(max, Math.abs(ArmsForward));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;


        }

        // Send calculated power to wheels
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);
        opMode.telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
        opMode.telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);

    }
    public void init_encoders (){
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public void setPosition ( int Pos){
//        leftBackDrive.setTargetPosition(Pos);
//        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftFrontDrive.setTargetPosition(Pos);
//        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightBackDrive.setTargetPosition(Pos);
//        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightFrontDrive.setTargetPosition(Pos);
//        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }
    public boolean isBusy(){

        return leftBackDrive.isBusy();

    }

    public int getPosition() {
        return leftBackDrive.getCurrentPosition();
    }
}
