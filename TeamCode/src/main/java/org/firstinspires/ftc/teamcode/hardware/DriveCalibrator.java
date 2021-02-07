package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "CRUSADE", group = "Autonomous")
public class DriveCalibrator extends LinearOpMode {

    Launcher launcher;

    public void runOpMode(){
        launcher = new Launcher(hardwareMap, telemetry);
//        launcher.elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        waitForStart();
        while (!isStopRequested()) { //tick -890
            if (gamepad1.x) {
//                launcher.elevator.setTargetPosition(890);
//                launcher.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                launcher.elevator.setPower(0.7);
                sleep(500);
            }
            else if (gamepad1.y){
//                launcher.elevator.setTargetPosition(0);
//                launcher.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                launcher.elevator.setPower(0.7);
                sleep(500);
            }
//            telemetry.addLine("elevat: " + launcher.elevator.getCurrentPosition());
            telemetry.update();

        }
    }
}
