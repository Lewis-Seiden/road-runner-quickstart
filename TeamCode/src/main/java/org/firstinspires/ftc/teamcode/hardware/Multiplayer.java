package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

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
        Gyro gyro = new Gyro(hardwareMap);
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbler.wobblerLifter.setTargetPosition(0);
        wobbler.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        launcher.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double fineTune = 1;
        boolean wobbleOpen = true;
        boolean ringIn = false;
        boolean endgame = false;

        waitForStart();
        start();
//        launcher.runDaRunner(1);
        while (opModeIsActive()) {
            mecanumDrive.SetPowerMecanum(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, fineTune);
            if (gamepad2.b) {
                wobbler.wobblerClose();
                wobbleOpen = false;
//                telemetry.addLine("Wobbler Closeing");
            }
            if (gamepad2.a) {
                wobbler.wobblerOpenDC();
                wobbleOpen = true;
//                telemetry.addLine("Wobbler Opening");
            }
            if (gamepad2.left_stick_y > 0.05 && !wobbleOpen) {
                wobbler.wobblerDown();
            } else if (gamepad2.left_stick_y < -0.05 && !wobbleOpen) {
                wobbler.wobblerFull();
            }

            if(gamepad2.y) {
//                launcher.inTake.setPower(-1);
                launcher.inTake2.setPower(-1);
            } else if((gamepad2.right_trigger > 0.1  || endgame) && !gamepad2.x){
                launcher.inTake.setPower(0);
                launcher.inTake2.setPower(0);
            } else {
                launcher.inTake.setPower(1);
                launcher.inTake2.setPower(1);
            }

            if (gamepad2.right_trigger > 0.1 && !endgame) {
                if (gamepad2.left_trigger > 0.1) {

                    launcher.SpinFlywheel(0.55);
                }
                else {
                    launcher.SpinFlywheel(1);
                }

            } else {
                launcher.SpinFlywheel(0);

            }
//            if(gamepad1.a)
//            {
//                double integral = 0;
//                double previousError = gyro.getAngle();
//                ElapsedTime time = new ElapsedTime();
//                double prevTime = time.milliseconds()/1000;
//                double KP = 1;
//                double KI = 0;
//                double KD = 0;
//                double output;
//                while(true) {
//                    double error = -gyro.getAngle();
//                    double currentTime = time.milliseconds()/1000;
//                    double deltatime = currentTime - prevTime;
//                    prevTime = currentTime;
//                    integral += error * deltatime;
//                    double derivative = (error - previousError)/deltatime;
//                    previousError = error;
//                    output = KP * error + KI * integral + KD * derivative;
//                    mecanumDrive.SetPowerMecanum(0, 0, output, 1);
//
//                }
//
//
//            }
            if (gamepad2.right_trigger > 0.1 && endgame) {
                launcher.SpinFlywheel(1);
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
            if(endgame && gamepad2.right_trigger > 0.1) {
                launcher.inTake.setPower(0);
                launcher.inTake2.setPower(0);
            }
            //Makes the robot shoot 3 ring
            telemetry.addLine("isEndgame: " + endgame);
            if (gamepad2.right_bumper && !endgame) {
                mecanumDrive.SetPowerMecanum(0,0,0,1);
                for(int i = 0; i < 3; i++) {
                    launcher.MovePusher(0.3);
                    sleep(400);
                    launcher.MovePusher(0);
                    sleep(400);
                }

            } else if (gamepad2.right_bumper && endgame) {
                launcher.MovePusher(0.3);
                sleep(400);
                launcher.MovePusher(0);
            }
             else {
                launcher.MovePusher(0);
//                telemetry.addLine("Shooter pusher not pushing");
            }

            if (gamepad2.left_bumper){
                launcher.ringBlocker.setPosition(0.6);
            }else if (endgame){
                launcher.ringBlocker.setPosition(0);
            }else {
                launcher.ringBlocker.setPosition(0.4);
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
