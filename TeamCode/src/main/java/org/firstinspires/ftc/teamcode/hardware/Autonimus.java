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
@Disabled
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
        sleep(200);
        int x = testPipeline.stack;
        if (x == 4) {
            x = 2;
        }
        wobble.wobblerClose();
        wobble.wobblerFull();
        int xShoot = 72;
        int yShoot = 50;
        Trajectory dropWobbler;
        if(x == 0) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(72, 0), Math.toRadians(0))
                    .build();
        }
        if(x == 1) {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d( 108, 0), Math.toRadians(0))
                    .build();
        }
        else {

            dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())
                    .splineToConstantHeading(new Vector2d(144, 1), Math.toRadians(0))
                    .build();
        }
        Trajectory shootPosition = mecanumDrive.trajectoryBuilder(dropWobbler.end())
                .splineToConstantHeading(new Vector2d(1,1), Math.toRadians(0))
                .build();

        while (!isStarted()) {
            telemetry.addData("stack", testPipeline.stack);
            telemetry.update();
        }

        webcam.stopStreaming();
            webcam.closeCameraDevice();


        mecanumDrive.followTrajectory(dropWobbler);

        wobble.wobblerDown();
        wobble.wobblerOpen();
        wobble.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);

        mecanumDrive.followTrajectory(shootPosition);

        //spin up shooter, and then shoot
        launcher.SpinFlywheel(1);
        sleep(1000);
        for (int i = 0; i < 3; i++) {
            launcher.MovePusher(0.2);
            sleep(1000);
            launcher.MovePusher(0);
            sleep(1000);
        }






   }
}
//ingore