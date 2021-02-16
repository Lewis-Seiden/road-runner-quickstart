package org.firstinspires.ftc.teamcode.vision;
//

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

//@Disabled
@TeleOp(name = "vision test", group = "TeleOp")
public class VisionTester extends LinearOpMode {
    public void runOpMode () {
        waitForStart();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        TestPipeline testPipeline = new TestPipeline();
        webcam.setPipeline(testPipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240);
            while (!isStopRequested()) {
                testPipeline.x += gamepad1.left_stick_x * 0.05;
                testPipeline.y += gamepad1.left_stick_y * 0.05;
                testPipeline.width += gamepad2.right_stick_x * 0.05;
                testPipeline.height += gamepad2.right_stick_y * 0.05;
                //Makes it possible to change the size and location of the boxes when running vision tester
                telemetry.addData("HSV mean",testPipeline.hsvValue);
                telemetry.addLine("x: "+testPipeline.x+", y: "+testPipeline.y+", width: "+testPipeline.width+", height: "+testPipeline.height);
                //Makes phone show coordinates and hsv mean
                telemetry.update();
                sleep(10);

        }
    }



}
