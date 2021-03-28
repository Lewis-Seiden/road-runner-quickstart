package org.firstinspires.ftc.teamcode.drive.oldAuto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.hardware.AutoMethods;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;

//STUFF IS COMMENTED FOR TESTING, UNCOMMENT BEFORE USE

@Autonomous(name="AUTOTEST", group="Linear Opmode")
//@Disabled
public class RoadrunnerTests extends AutoMethods {
    public double xMove = -80.0;
    public double yMove = 0.0;
    public double headingTarget = 0.0;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() {

        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        wobble = new Wobbler(hardwareMap, telemetry);
//        launcher = new Launcher(hardwareMap, telemetry);
//        wobble.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        sleep(200);


        Trajectory test;
        test = mecanumDrive.trajectoryBuilder(new Pose2d())
                .splineToConstantHeading(new Vector2d( xMove, yMove), Math.toRadians(headingTarget))
                .build();
        waitForStart();
        mecanumDrive.followTrajectory(test);

    }
}