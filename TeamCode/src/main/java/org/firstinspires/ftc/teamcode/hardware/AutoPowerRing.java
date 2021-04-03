package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

@Autonomous(name="AUTOPWRRING", group="Linear Opmode")
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
//        sleep(10000);
        int x = testPipeline.stack;
        x = 4;
        int xShoot = 40;
        int yShoot = -14;
        int headingShoot = -8;
        Trajectory dropWobbler;
        Trajectory dropSecondWobbler;
        Trajectory ringPickUp;
        wobble.wobblerClose();
        wobble.wobblerFull();


        launcher.intakeFlipperRun(0);
        if (x == 0) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToLinearHeading(new Pose2d(72, 0, Math.toRadians(90)), Math.toRadians(0))
                    .build();
        } else if (x == 1) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToSplineHeading(new Pose2d(70, 2, 0), Math.toRadians(0))
                    .splineToSplineHeading(new Pose2d(85, -25, Math.toRadians(-90)), Math.toRadians(-90))
                    .build();
            xShoot = 40;
            yShoot = -14;
        } else {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, -5), Math.toRadians(0))
                    .build();
            xShoot = 34;
            yShoot = -24;
            headingShoot = -2;
        }

        Trajectory wobbleLeave = mecanumDrive.trajectoryBuilder(dropWobbler.end())
                .strafeRight(10)
                .build();
        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(wobbleLeave.end(), true)
                .splineToLinearHeading(new Pose2d(xShoot, yShoot, Math.toRadians(headingShoot)), Math.toRadians(180))
                .build();

        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL/2, TRACK_WIDTH)));
        if (x == 1) {
            ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end())
                    .splineToConstantHeading(new Vector2d(30, -14), Math.toRadians(180))
                    .build();
        } else if (x == 4){
            ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end().plus(new Pose2d(0,0, Math.toRadians(-110))))
                    .lineTo(new Vector2d(35, -3))
                    .build();
            xShoot = 38;
            yShoot = -14;
            headingShoot = -10;
        } else {
            ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end())
                    .build();
        }
        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)));

        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(ringPickUp.end())
                .splineToLinearHeading(new Pose2d(xShoot, yShoot, Math.toRadians(headingShoot)), Math.toRadians(0))
                .build();


        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition2.end())
                .splineToSplineHeading(new Pose2d(28, -13, Math.toRadians(90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(23, -13, Math.toRadians(90)), Math.toRadians(180))
                .build();
        if (x == 0) {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())
                    .splineToConstantHeading(new Vector2d(72, -5), Math.toRadians(0))
                    .build();
        } else if (x == 1) {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())
                    .splineToLinearHeading(new Pose2d(71, -18, Math.toRadians(-90)), Math.toRadians(180))
                    .build();
        } else {
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, -5), Math.toRadians(0))
                    .build();
        }
        Trajectory line = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end(), true)
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(70, -30), Math.toRadians(180))
                .build();
//        telemetry.addLine("hi");


//        while (!isStarted() && opModeIsActive()) {
//            telemetry.addData("stack ", x);
//            telemetry.update();
//        }

        waitForStart();

        webcam.stopStreaming();
        webcam.closeCameraDevice();



//        wobble.wobblerUp();
        launcher.SpinFlywheel(1);

        wobble.wobblerUp();
        mecanumDrive.followTrajectory(dropWobbler);
        wobble.wobblerDown();
        sleep(100);
        wobble.wobblerOpen();

        mecanumDrive.followTrajectory(wobbleLeave);

        mecanumDrive.followTrajectory(shootPosition1);

        launcher.MovePusher(0.2);
        sleep(1000);
        launcher.MovePusher(0);
        sleep(1000);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        sleep(700);
        launcher.MovePusher(0.2);
        sleep(300);



        if (x == 1) {
            launcher.inTake.setPower(1);
            launcher.inTake2.setPower(1);
            sleep(100);
            mecanumDrive.followTrajectory(ringPickUp);
            sleep(400);
            launcher.inTake.setPower(0);
            launcher.inTake2.setPower(0);
            launcher.MovePusher(0);
            mecanumDrive.followTrajectory(shootPosition2);
            launcher.MovePusher(0.2);
            sleep(700);
            launcher.MovePusher(0);

        }
        else if (x == 4)
        {
            mecanumDrive.turn(Math.toRadians(-110 - mecanumDrive.getPoseEstimate().getHeading()));
            launcher.inTake.setPower(1);
            launcher.inTake2.setPower(1);
            launcher.MovePusher(0);
            sleep(100);
            mecanumDrive.followTrajectory(ringPickUp);
            sleep(400);
            launcher.inTake.setPower(0);
            launcher.inTake2.setPower(0);
            mecanumDrive.followTrajectory(shootPosition2);
            launcher.MovePusher(0.2);
            sleep(700);
            launcher.MovePusher(0);
            sleep(700);
            launcher.MovePusher(0.2);
            sleep(700);
            launcher.MovePusher(0);
            sleep(700);
            launcher.MovePusher(0.2);
            sleep(300);
            launcher.MovePusher(0);
    }


        //spin up shooter, and then shoot
        wobble.wobblerDown();

        mecanumDrive.followTrajectory(secondWobble);
        sleep(300);
        wobble.wobblerClose();
        sleep(100);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(200);

        mecanumDrive.followTrajectory(line);
        launcher.intakeFlipperRun(0);
    }




}
