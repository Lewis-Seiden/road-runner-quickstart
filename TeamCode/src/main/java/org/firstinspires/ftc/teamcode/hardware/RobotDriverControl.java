package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@TeleOp(name="Mecanumthejank", group="Linear Opmode")
public class RobotDriverControl extends LinearOpMode {

    //take inputs from game pad, gamepad1.left_trigger_x
    //use varibles to find power
    //call SetPowerMecanum()


    @Override

    public void runOpMode(){
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);


        Wobbler wobbler = new Wobbler(hardwareMap, telemetry);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.wobblerLifter.setTargetPosition(0);
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        launcher.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double fineTune = 1;
//        ElevatorMoveEventManager move = new ElevatorMoveEventManager(launcher, this);
        waitForStart();
     //   move.start();
//        launcher.runDaRunner(1);
        while(opModeIsActive()) {
            mecanumDrive.SetPowerMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, fineTune);
            if(gamepad1.b)
            {
                wobbler.wobblerClose();
                telemetry.addLine("Wobbler Closeing");
                telemetry.update();
            }
            if(gamepad1.a)
            {
                wobbler.wobblerOpen();
                telemetry.addLine("Wobbler Opening");
                telemetry.update();
            }
            if (gamepad1.x) {
       //         move.elevatorMove();
            }
            if (gamepad1.dpad_up)
            {
                wobbler.wobblerUp();
                telemetry.addLine("Wobbler Going Up. Encoder Value: " + wobbler.wobblerLifter.getCurrentPosition());
                telemetry.update();
            }
            else if(gamepad1.dpad_down)
            {
                wobbler.wobblerDown();
                telemetry.addLine("Wobbler Going Down. Encoder Value: " + wobbler.wobblerLifter.getCurrentPosition());
                telemetry.update();
            }
            else if(gamepad1.dpad_left || gamepad1.dpad_right)
            {
                wobbler.wobblerCarry();
                telemetry.addLine("Wobbler In Carry Position. Encoder Value: " + wobbler.wobblerLifter.getCurrentPosition());
                telemetry.update();
            }

            if (gamepad1.right_bumper){
                launcher.MovePusher(0.2);
                telemetry.addLine("Shooter pusher pushing");
                telemetry.update();

            } else {
                launcher.MovePusher(0);
                telemetry.addLine("Shooter pusher not pushing");
                telemetry.update();
            }

            if (gamepad1.right_trigger > 0.1){
                launcher.SpinFlywheel(1);
            } else {
                launcher.SpinFlywheel(0);
            }

            if (gamepad1.left_trigger > 0.1) {
                fineTune = 3;
            } else {
                fineTune = 1;
            }
//            if (gamepad1.left_bumper){
//                launcher.runIntake(1);
//            } else {
//                launcher.runIntake(0);
//            }
//            if (gamepad1.y){
//                launcher.runIntake(-1);
//            }

        }
    }
}

