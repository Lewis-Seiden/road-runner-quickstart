package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Wobbler;

public abstract class AutoMethods extends LinearOpMode {
    protected Wobbler wobble;

    protected Launcher launcher;
    final double TICKS_PER_INCH = 60.7873786408;
    protected SampleMecanumDrive mecanumDrive;

    public static double wrapAngle(double radians){
        while (radians > Math.toRadians(180)){
            radians -= Math.toRadians(360);
        }
        while (radians < Math.toRadians(-180)){
            radians += Math.toRadians(360);
        }
        return radians;
    }
}