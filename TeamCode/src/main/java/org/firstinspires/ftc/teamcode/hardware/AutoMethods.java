package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Wobbler;

public abstract class AutoMethods extends LinearOpMode {
    protected Wobbler wobble;
    protected MecanumDrive mecanumDrive;
    protected Launcher launcher;
    final double TICKS_PER_INCH = 60.7873786408;

    public void goStraightNoPid(double power, double distance) {
        //Takes current position for each motor
        double LFinitial = mecanumDrive.leftFrontDrive.getCurrentPosition();
        double LBinitial = mecanumDrive.leftBackDrive.getCurrentPosition();
        double RFinitial = mecanumDrive.rightFrontDrive.getCurrentPosition();
        double RBinitial = mecanumDrive.rightBackDrive.getCurrentPosition();
        //Calculates how far it needs to go
        double LFtarget = LFinitial - TICKS_PER_INCH * distance;
        double LBtarget = LBinitial - TICKS_PER_INCH * distance;
        double RFtarget = RFinitial - TICKS_PER_INCH * distance;
        double RBtarget = RBinitial - TICKS_PER_INCH * distance;

        //sets that as its target
//        mecanumDrive.leftFrontDrive.setTargetPosition((int)LFtarget);
//        mecanumDrive.leftBackDrive.setTargetPosition((int)LBtarget);
//        mecanumDrive.rightFrontDrive.setTargetPosition((int)RFtarget);
//        mecanumDrive.rightBackDrive.setTargetPosition((int)RBtarget);
        //goes there
        boolean yesLF = true;
        boolean yesLB = true;
        boolean yesRF = true;
        boolean yesRB = true;

        mecanumDrive.leftFrontDrive.setPower(power * -1);
        mecanumDrive.leftBackDrive.setPower(power * -1);
        mecanumDrive.rightFrontDrive.setPower(power * -1);
        mecanumDrive.rightBackDrive.setPower(power * -1);

        while ((yesLB || yesLF || yesRB || yesRF) && !isStopRequested()) {
            if (Math.abs(mecanumDrive.leftFrontDrive.getCurrentPosition() - LFtarget) < 100) {
                mecanumDrive.leftFrontDrive.setPower(0);
            }
            if (Math.abs(mecanumDrive.leftBackDrive.getCurrentPosition() - LBtarget) < 100) {
                mecanumDrive.leftBackDrive.setPower(0);
            }
            if (Math.abs(mecanumDrive.rightFrontDrive.getCurrentPosition() - RFtarget) < 100) {
                mecanumDrive.rightFrontDrive.setPower(0);
            }
            if (Math.abs(mecanumDrive.rightBackDrive.getCurrentPosition() - RBtarget) < 100) {
                mecanumDrive.rightBackDrive.setPower(0);
            }
        }

        mecanumDrive.leftFrontDrive.setPower(0);
        mecanumDrive.leftBackDrive.setPower(0);
        mecanumDrive.rightFrontDrive.setPower(0);
        mecanumDrive.rightBackDrive.setPower(0);
//        mecanumDrive.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mecanumDrive.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mecanumDrive.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        mecanumDrive.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //


    }
}