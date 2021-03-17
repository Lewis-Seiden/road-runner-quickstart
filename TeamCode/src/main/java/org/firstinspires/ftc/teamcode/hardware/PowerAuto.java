package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.hardware.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

//STUFF IS COMMENTED FOR TESTING, UNCOMMENT BEFORE USE

@Autonomous(name="AUTOPWR", group="Linear Opmode")
//@Disabled
public class PowerAuto extends AutoMethods {

    public void runOpMode() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble = new Wobbler(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        wobble.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        UltamiteGoalPipeline testPipeline = new UltamiteGoalPipeline();
        webcam.setPipeline(testPipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240);
        sleep(10000);

        int x = testPipeline.stack;
        // int x=2;
        if (x == 4) {
            x = 2;
        }



        wobble.wobblerClose();
        wobble.wobblerFull();
        Trajectory dropWobbler;
        Trajectory dropSecondWobbler;

        wobble.wobblerUp();
        launcher.intakeFlipperRun(0);
        int y = 0;
        int z = 0;
        int a = 0;
        if (x == 2) {
            z = 10;
        }
        else if(x == 1)
        {
            z = 7;
        }
        else if (x == 0);
        {
            z = 8;
        }
        if(x == 0)
        {
            y = 0;
        }
        else if (x == 1)
        {
            y = 7;
        }
        else if (x == 2)
        {
            y = 7;
        }
        if(x == 4)
        {
           a = -2;
        }

        if(x == 0) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(72, 0), Math.toRadians(0))
                    .build();
        }
        else if(x == 1) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(96, -26), Math.toRadians(0))
                    .build();
        }
        else {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, 0), Math.toRadians(0))
                    .build();
        }
        //make dropwobbler2 from secondwobbler.end
        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(dropWobbler.end())
                .lineToLinearHeading(new Pose2d(56,-34.5 + z, Math.toRadians(-7)))
                .build();
        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(shootPosition1.end())
                .lineToLinearHeading(new Pose2d(56,-43 +  z, Math.toRadians(-10)))
                .build();
        Trajectory shootPosition3 = mecanumDrive.trajectoryBuilder(shootPosition2.end())
                .lineToLinearHeading(new Pose2d(56,-52.5 + z, Math.toRadians(-10)))
                .build();
        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition3.end(),true)
                .splineToLinearHeading(new Pose2d(25 + a, -26 + y, Math.toRadians(90)), Math.toRadians(180))
                .build();

        if(x == 0) {
//set up second wobble drop
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())
                    .splineToLinearHeading(new Pose2d(72, -10, Math.toRadians(0)), Math.toRadians(0))
                    .build();
        }
        else if(x == 1) {

            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(96, -28), Math.toRadians(0))
                    .build();
        }
        else {

            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(115, 0), Math.toRadians(0))
                    .build();

        }

        Trajectory endPosition = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end(), true)
                .splineToConstantHeading(new Vector2d(70, -30), Math.toRadians(180))
                .build();


        while (!isStarted()) {
            telemetry.addData("stack ", x);
            telemetry.update();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();

        wobble.wobblerDown();
            mecanumDrive.followTrajectory(dropWobbler);
        launcher.SpinFlywheel(0.9);
        wobble.wobblerOpen();
        wobble.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
            mecanumDrive.followTrajectory(shootPosition1);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
            mecanumDrive.followTrajectory(shootPosition2);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
            mecanumDrive.followTrajectory(shootPosition3);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);

        //spin up shooter, and then shoot

        mecanumDrive.followTrajectory(secondWobble);
        sleep(1000);
        wobble.wobblerClose();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(500);

        mecanumDrive.followTrajectory(endPosition);
        launcher.intakeFlipperRun(-1);
    }
}