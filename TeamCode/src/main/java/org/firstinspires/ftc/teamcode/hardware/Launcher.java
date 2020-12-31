package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static android.os.SystemClock.sleep;

public class Launcher {
    private Servo pusher;
    private Servo elevatorFlicker;
    private DcMotor launcherFlywheel;
    private Telemetry telemetry;
    private DcMotor inTake;
    public DcMotor elevator;
    public CRServo runner;
    public TouchSensor ringSensor;
    private HardwareMap hardwareMap;
    //private Servo push1;
    //private Servo push2;

    public Launcher(HardwareMap hardwareMap, Telemetry t) {
        this.hardwareMap = hardwareMap;
        launcherFlywheel = hardwareMap.get(DcMotor.class, "launcher_flywheel");
        pusher = hardwareMap.get(Servo.class, "pusher");
        elevatorFlicker = hardwareMap.get(Servo.class, "elevator_flicker");
        inTake = hardwareMap.get(DcMotor.class, "intake");
        elevator = hardwareMap.get(DcMotor.class, "elevator");
        pusher.setDirection(Servo.Direction.REVERSE);
        runner = hardwareMap.get(CRServo.class, "roller");
        ringSensor = hardwareMap.get(TouchSensor.class, "ring_sensor");
        //push1 = hardwareMap.get(Servo.class, "push1");
        //push2 = hardwareMap.get(Servo.class, "push2");


    }

    //double -1 fully backwards power to 1 fully forwards power for motor power
    public void MovePusher(double postition) {
        pusher.setPosition(postition);
    }

    //double -1 fully backwards power to 1 fully forwards power for motor power
    public void SpinFlywheelNoVoltageControl(double speed) {
        launcherFlywheel.setPower(speed);
    }

    public void runIntake(double speed) {
        inTake.setPower(speed);
    }

    public void elevatorMove(double speed, int offset) {
        // target = 6785 for up
        //push1.setPosition(0);
        //push1.setPosition(0);
        elevatorFlicker.setPosition(0.2);
        elevator.setTargetPosition(-1186);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setPower(speed);
        sleep(500);
        elevatorFlicker.setPosition(1);
        sleep(1000);
        elevatorFlicker.setPosition(0);
        sleep(100);
        elevator.setTargetPosition(offset);
        if(elevator.getCurrentPosition() > -117){
            int dif = elevator.getTargetPosition() - elevator.getCurrentPosition();
            elevator.setPower(speed * dif * 0.005);
        }

        //push1.setPosition(1);
        //push1.setPosition(1);
    }
    public void runDaRunner(double speed)
    {
        runner.setPower(speed);
    }

    public void SpinFlywheel(double speed){
        launcherFlywheel.setPower(11.5/ Voltage.getVoltage(hardwareMap) * speed);//change the number to change target voltage
    }

}