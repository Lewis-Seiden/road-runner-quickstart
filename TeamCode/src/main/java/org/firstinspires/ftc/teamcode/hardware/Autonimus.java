package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

//STUFF IS COMMENTED FOR TESTING, UNCOMMENT BEFORE USE

@Autonomous(name="AUTO", group="Linear Opmode")
//@Disabled
public class Autonimus extends AutoMethods {

    public void runOpMode() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        wobble = new Wobbler(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        boolean pwrShots = false; //true = go for pwr shots, false = go for high goal
        wobble.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
        UltamiteGoalPipeline testPipeline = new UltamiteGoalPipeline();
        webcam.setPipeline(testPipeline);
        webcam.openCameraDevice();
        webcam.startStreaming(320,240);
        sleep(1000);


        int x = testPipeline.stack;
           // int x=2;
        if (x == 4) {
            x = 2;
        }

        int twoRingPickUpX = 0;
        double twoRingPickUpY = 0;
        if(x == 1){
             twoRingPickUpX = 23;
             twoRingPickUpY = -20.5;
        }
        else if (x == 2)
        {

            twoRingPickUpX = 23;
            twoRingPickUpY = -15;
        }
        else {
             twoRingPickUpX = 25;
             twoRingPickUpY = -20.0;


        }

        wobble.wobblerClose();
        wobble.wobblerFull();
        int xShoot = 37;
        int yShoot = -14;
        Trajectory dropWobbler;
        Trajectory dropSecondWobbler;

        launcher.SpinFlywheel(1);
        wobble.wobblerUp();

        if(x == 0) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(72, 0), Math.toRadians(0))
                    .build();
        }
        else if(x == 1) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(96, -29), Math.toRadians(0))
                    .build();
        }
        else {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(120, 0), Math.toRadians(0))
                    .build();
        }
        //make dropwobbler2 from secondwobbler.end
        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(dropWobbler.end(), true)
                .splineToLinearHeading(new Pose2d(xShoot + 2,yShoot, Math.toRadians(-7)), Math.toRadians(0))
                .build();
        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(shootPosition1.end(), true)
                .splineToLinearHeading(new Pose2d(xShoot,yShoot, Math.toRadians(-7)), Math.toRadians(0))
                .build();
        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition2.end(),false)
                .splineToLinearHeading(new Pose2d(37, -22, Math.toRadians(60)), Math.toRadians(0))
                .build();
        Trajectory secondWobblePt2 = mecanumDrive.trajectoryBuilder(secondWobble.end(), true)
                .splineToLinearHeading(new Pose2d(twoRingPickUpX, twoRingPickUpY, Math.toRadians(90)), Math.toRadians(0))
                .build();

        if(x == 0) {
//set up second wobble drop
            dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobblePt2.end())
                    .splineToLinearHeading(new Pose2d(72, -10, Math.toRadians(0)), Math.toRadians(0))
                    .build();
        }
        else if(x == 1) {

            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(96, -34), Math.toRadians(0))
                    .build();
        }
        else {

            dropSecondWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(115, -5), Math.toRadians(0))
                    .build();
        }

        Trajectory endPosition = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end())
                .splineToConstantHeading(new Vector2d(65, -30), Math.toRadians(0))
                .build();


        while (!isStarted()) {
            telemetry.addData("stack ", x);
            telemetry.update();
        }

        webcam.stopStreaming();
            webcam.closeCameraDevice();

        wobble.wobblerDown();
        mecanumDrive.followTrajectory(dropWobbler);

        wobble.wobblerOpen();
        wobble.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
        mecanumDrive.followTrajectory(shootPosition1);
        launcher.MovePusher(0.2);
        sleep(1000);
        launcher.MovePusher(0);
        sleep(1000);
        mecanumDrive.followTrajectory(shootPosition2);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        sleep(700);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        sleep(700);

        //spin up shooter, and then shoot



        mecanumDrive.followTrajectory(secondWobble);
        mecanumDrive.followTrajectory(secondWobblePt2);
        sleep(1000);
        wobble.wobblerClose();
        wobble.wobblerUp();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(500);

        mecanumDrive.followTrajectory(endPosition);
   }
}