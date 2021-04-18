package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

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

@Autonomous(name="AUTORING", group="Linear Opmode")
//@Disabled
public class AutoRing extends AutoMethods {

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
        stack = testPipeline.stack;
        launcher.intakeFlipperRun(0);
        launcher.ringBlocker.setPosition(1);

        if(stack == 0){
            autoStack0();
        } else if (stack == 1){
            autoStack1();
        } else {
            autoStack4();
        }




    }
}
