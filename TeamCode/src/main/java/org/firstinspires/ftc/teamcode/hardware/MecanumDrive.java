package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

//Controls everything related to the drive train
public class MecanumDrive {
    public DcMotor leftFrontDrive;
    public DcMotor rightFrontDrive;
    public DcMotor leftBackDrive;
    public DcMotor rightBackDrive;

    public MecanumDrive(HardwareMap hardwareMap)
    {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        //sets which way the motors run
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void SetPowerMecanum(double forwardPower, double sidePower, double rotationPower, double fineTune){ //Makes the thing move
        //sidePower right is positive
        //rotationPower clockwise is positive

        double leftFrontPower = (-forwardPower + sidePower + rotationPower);
        double rightFrontPower = (forwardPower + sidePower + rotationPower);
        double leftBackPower = (forwardPower + sidePower - rotationPower);
        double rightBackPower = (-forwardPower + sidePower - rotationPower);
        double maximumPower = Math.abs(leftFrontPower);

        //Power is between -1 and 1

        if (Math.abs(rightFrontPower) > maximumPower) {
            maximumPower = Math.abs(rightFrontPower);
        }
        if (Math.abs(leftBackPower) > maximumPower) {
            maximumPower = Math.abs(leftBackPower);
        }
        if(Math.abs(rightBackPower) > maximumPower){
            maximumPower = Math.abs(rightBackPower);
        }

        if(maximumPower > 1){
            leftFrontPower /= maximumPower;
            leftBackPower /= maximumPower;
            rightFrontPower /= maximumPower;
            rightBackPower /= maximumPower;
        }
        leftFrontDrive.setPower(leftFrontPower/fineTune);
        rightFrontDrive.setPower(rightFrontPower/fineTune);
        leftBackDrive.setPower(leftBackPower/fineTune);
        rightBackDrive.setPower(rightBackPower/fineTune);




    }


}
