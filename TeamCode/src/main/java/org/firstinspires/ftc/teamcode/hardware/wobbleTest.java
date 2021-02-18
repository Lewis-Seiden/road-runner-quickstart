package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Wobbler Tuner", group = "TeleOp")
public class wobbleTest extends LinearOpMode {


    public void runOpMode() throws InterruptedException {
        Wobbler wobbler = new Wobbler(hardwareMap, telemetry);
        while (!isStopRequested()) {

            telemetry.addLine(wobbler.wobblerLifter.getCurrentPosition() + " ");
            telemetry.update();
        }
    }
}
