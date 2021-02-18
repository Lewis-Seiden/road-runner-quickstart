package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Wobbler {
   private Servo wobblerGrabber;
   public DcMotor wobblerLifter;
   private Telemetry telemetry;

   private int encoderTicks = 3892;
   private double gearRatio = 2;
   private double encoderTicksPerDegree = encoderTicks/360 * gearRatio;

    public Wobbler(HardwareMap hardwareMap, Telemetry t)
    {
        wobblerGrabber = hardwareMap.get(Servo.class, "wobbler_grabber");
        wobblerLifter = hardwareMap.get(DcMotor.class, "wobbler_lifter");
        this.telemetry = t;
    }

    public void wobblerOpen(){
        wobblerGrabber.setPosition(0.5);
//        telemetry.addLine("Wobbler open function called.");
    }

    public void wobblerOpenDC(){
        wobblerGrabber.setPosition(0.3);
//        telemetry.addLine("Wobbler open function called.");
    }
    //robot releases the wobbler
    public void wobblerClose(){
        wobblerGrabber.setPosition(0);
//        telemetry.addLine("Wobbler closed function called.");
    }

    //0 is all the way down, 150 is up

    //robot grabs the wobbler
    public void wobblerDown(){

            wobblerLifter.setTargetPosition(1);
            wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wobblerLifter.setPower(1);

//        telemetry.addLine("Wobbler down function called.");
    }
    //wobbler gets put down
    public void wobblerFull() {
//        wobblerLifter.setTargetPosition((int)(-150 * encoderTicksPerDegree));
        wobblerLifter.setTargetPosition(wobblerLifter.getCurrentPosition() - 100);
        wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobblerLifter.setPower(-0.3);
//        telemetry.addLine("Wobbler up function called.");
        telemetry.update();
//        t.addLine(encoder.pos);

    }
    public void wobblerUp() {
//        wobblerLifter.setTargetPosition((int)(-150 * encoderTicksPerDegree));
        wobblerLifter.setTargetPosition(-2848);
        wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobblerLifter.setPower(1);

//        telemetry.addLine("Wobbler up function called.");

//        t.addLine(encoder.pos);

    }
    //wobbler gets pulled up or down to a neutral position
    public void wobblerCarry(){
//         wobblerLifter.setTargetPosition((int)(-5 * encoderTicksPerDegree));
         wobblerLifter.setTargetPosition(-650);
         if(wobblerLifter.getCurrentPosition() > wobblerLifter.getTargetPosition()) {
             wobblerLifter.setPower(-0.2);
         } else {
             wobblerLifter.setPower(0.7);
         }
//         telemetry.addLine("Wobbler in carry position function called.");
         telemetry.update();

    }



}
