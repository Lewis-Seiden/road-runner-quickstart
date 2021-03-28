package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
@Autonomous(name="AUTORING", group="Linear Opmode")
public class AutoPowerRing extends AutoMethods {

    public void runOpMode () {
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
        webcam.startStreaming(320, 240);
        sleep(10000);
        int x = testPipeline.stack;
        x = 1;
        int xShoot = 38;
        int yShoot = -14;
        Trajectory dropWobbler;
        Trajectory dropSecondWobbler;
        wobble.wobblerClose();
        wobble.wobblerFull();


        launcher.intakeFlipperRun(0);
        if (x == 0) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(72, 0), Math.toRadians(0))
                    .build();
        } else if (x == 1) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(70, 2), Math.toRadians(0))
                    .splineToConstantHeading(new Vector2d(96, -29), Math.toRadians(-90))
                    .build();
        } else {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, 0), Math.toRadians(0))
                    .build();
        }
        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(dropWobbler.end(), true)
                .splineToLinearHeading(new Pose2d(xShoot + 1, yShoot, Math.toRadians(-8)), Math.toRadians(180))
                .build();

        Trajectory ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end(), true)
                .splineToConstantHeading(new Vector2d(30, -12), Math.toRadians(140))
                .build();
        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(ringPickUp.end())
                .splineToLinearHeading(new Pose2d(xShoot + 1, yShoot, Math.toRadians(-8)), Math.toRadians(0))
                .build();


        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition2.end())
                .splineToSplineHeading(new Pose2d(28, -20.5, Math.toRadians(90)), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(21, -21.5), Math.toRadians(180))
                .build();
        if (x == 0) {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())
                    .splineToConstantHeading(new Vector2d(72, -5), Math.toRadians(0))
                    .build();
        } else if (x == 1) {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())
                    .splineToLinearHeading(new Pose2d(70, -24, Math.toRadians(-90)), Math.toRadians(180))
                    .build();
        } else {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, -5), Math.toRadians(0))
                    .build();
        }
        Trajectory line = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end())
                .splineToConstantHeading(new Vector2d(70, -30), Math.toRadians(0))
                .build();




        while (!isStarted()) {
            telemetry.addData("stack ", x);
            telemetry.update();
        }

        webcam.stopStreaming();
        webcam.closeCameraDevice();


        //wobble.wobblerDown();
        wobble.wobblerUp();
        mecanumDrive.followTrajectory(dropWobbler);
        wobble.wobblerOpen();
        launcher.SpinFlywheel(1);
        //wobble.wobblerOpen();
        //wobble.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        mecanumDrive.followTrajectory(shootPosition1);
        launcher.MovePusher(0.2);
        sleep(1000);
        launcher.MovePusher(0);
        sleep(1000);
        //mecanumDrive.followTrajectory(shootPosition2);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        sleep(700);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        sleep(700);
        if (x == 1) {
            launcher.inTake.setPower(1);
            launcher.inTake2.setPower(1);
            sleep(100);
            mecanumDrive.followTrajectory(ringPickUp);
            sleep(400);
            mecanumDrive.followTrajectory(shootPosition2);
            launcher.MovePusher(0.2);
            sleep(700);
            launcher.MovePusher(0);

        }

        //spin up shooter, and then shoot
        wobble.wobblerDown();

        mecanumDrive.followTrajectory(secondWobble);
        sleep(1000);
        wobble.wobblerClose();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(500);

        mecanumDrive.followTrajectory(line);
        launcher.intakeFlipperRun(0);
    }




}
