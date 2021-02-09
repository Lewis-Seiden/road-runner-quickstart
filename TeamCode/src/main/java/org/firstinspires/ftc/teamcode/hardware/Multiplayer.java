package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="TELEOP2P", group="Linear Opmode")
public class Multiplayer extends LinearOpMode {

    //take inputs from game pad, gamepad1.left_trigger_x
    //use varibles to find power
    //call SetPowerMecanum()


    @Override
    public void runOpMode() {
        MecanumDrive mecanumDrive = new MecanumDrive(hardwareMap);


        Wobbler wobbler = new Wobbler(hardwareMap, telemetry);
        Launcher launcher = new Launcher(hardwareMap, telemetry);
        Voltage voltage = new Voltage();
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.wobblerLifter.setTargetPosition(0);
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        launcher.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double fineTune = 1;
        boolean ringIn = false;
        boolean endgame = false;
        waitForStart();
        start();
//        launcher.runDaRunner(1);
        while (opModeIsActive()) {
            mecanumDrive.SetPowerMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, fineTune);
            if (gamepad2.b) {
                wobbler.wobblerClose();
//                telemetry.addLine("Wobbler Closeing");
            }
            if (gamepad2.a) {
                wobbler.wobblerOpenDC();
//                telemetry.addLine("Wobbler Opening");
            }
            if (gamepad2.left_stick_y > 0.05) {
                wobbler.wobblerDown();
            } else if (gamepad2.left_stick_y < -0.05) {
                wobbler.wobblerUp();
            }



            if (gamepad2.right_trigger > 0.1) {
                if (gamepad2.left_trigger > 0.1) {
                    launcher.SpinFlywheel(0.56);
                }
                else {
                    launcher.SpinFlywheel(1);
                }
                launcher.inTake.setPower(0);
            } else {
                launcher.SpinFlywheel(0);
                if(gamepad2.y){
                    launcher.inTake.setPower(12.6/ Voltage.getVoltage(hardwareMap) * -1);
                } else{
                    launcher.inTake.setPower(12.6/ Voltage.getVoltage(hardwareMap) * 1);
                }
            }

            if (gamepad1.left_trigger > 0.1) {
                fineTune = 3;
            } else {
                fineTune = 1;
            }

            if (gamepad2.dpad_right || gamepad2.dpad_left || gamepad2.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left || gamepad1.dpad_down ){
                endgame = true;
            } else if (gamepad2.dpad_up || gamepad1.dpad_up){
                endgame = false;
            }
            if (gamepad2.right_bumper && !endgame) {
                mecanumDrive.SetPowerMecanum(0,0,0,1);
                for(int i = 0; i < 3; i++) {
                    launcher.MovePusher(0.3);
                    sleep(400);
                    launcher.MovePusher(0);
                    sleep(400);
                }

            } else if (gamepad2.right_bumper && endgame){
                launcher.MovePusher(0.2);
            } else {
                launcher.MovePusher(0);
//                telemetry.addLine("Shooter pusher not pushing");
            }






            //        if(gamepad2.right_stick_y > 0.1){
            //            move.offset += 1;
            //            telemetry.addLine("elevator nudge up");
            //        } else if (gamepad2.right_stick_y < -0.1){
            //            move.offset -= 1;
            //            telemetry.addLine("elevator nudge down");
            //        }
            //        telemetry.addLine("offset " + move.offset);
            ringIn = launcher.ringSensor.isPressed();

            //if (!move.elevatorMoveRequsted){
            //            launcher.elevator.setTargetPosition(move.offset);
            //            launcher.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //            launcher.elevator.setPower(0.5);
            //        }
            telemetry.update();
        }
    }
}
