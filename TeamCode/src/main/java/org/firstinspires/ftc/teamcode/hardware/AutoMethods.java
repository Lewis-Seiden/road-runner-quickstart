package org.firstinspires.ftc.teamcode.hardware;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Gyro;
import org.firstinspires.ftc.teamcode.hardware.Launcher;
import org.firstinspires.ftc.teamcode.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.Wobbler;

import java.util.Arrays;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;

public abstract class AutoMethods extends LinearOpMode {
    protected Wobbler wobble;
    protected Launcher launcher;
    final double TICKS_PER_INCH = 60.7873786408;
    protected SampleMecanumDrive mecanumDrive;
    int stack;
    String tag = "autoDebug";

    public static double wrapAngle(double radians){
        while (radians > Math.toRadians(180)){
            radians -= Math.toRadians(360);
        }
        while (radians < Math.toRadians(-180)){
            radians += Math.toRadians(360);
        }
        return radians;
    }

    public void autoStack0(){

        //0 stack 2 wobbler

        mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Trajectory dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d()) //drops the wobbler
                .splineToLinearHeading(new Pose2d(72, 0, Math.toRadians(0)), Math.toRadians(0))
                .build();

        Trajectory wobbleLeave = mecanumDrive.trajectoryBuilder(dropWobbler.end()) //to make sure we dont knock it over
                .strafeRight(10)
                .build();

        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(wobbleLeave.end(), true) //goes to shoot
                .splineToSplineHeading(new Pose2d(36, -14, Math.toRadians(-8)), Math.toRadians(180))
                .build();

        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition1.end()) //grabs the second wobbler
                .splineToSplineHeading(new Pose2d(28, -20, Math.toRadians(90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(23.3, -20, Math.toRadians(90)), Math.toRadians(180))
                .build();

        Trajectory dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())//drops the second wobbler
                .splineToLinearHeading(new Pose2d(66, -8, Math.toRadians(0)), Math.toRadians(-180))
                .build();

        Trajectory linePark = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end(), true) //goes to the line
                .strafeRight(5)
                .splineToConstantHeading(new Vector2d(65, -30), Math.toRadians(180))
                .build();

        RobotLog.dd(tag,"init done");
        wobble.wobblerClose();
        wobble.wobblerFull();


        while (!isStarted() && !isStopRequested()) { //waits for it to start
            telemetry.addData("stack ", stack);
            telemetry.update();
            RobotLog.dd(tag, "waitforstart");
        }


        RobotLog.dd(tag,"start");


        launcher.SpinFlywheel(1);

        wobble.wobblerUp();
        mecanumDrive.followTrajectory(dropWobbler);
        wobble.wobblerDown();
        sleep(100);
        wobble.wobblerOpen();

        mecanumDrive.followTrajectory(wobbleLeave);

        RobotLog.dd(tag,"wobble1done"); //marks the moving and dropping of the first wobbler
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
        RobotLog.dd(tag,"shoot1done"); //marks the finishing of shooting the rings

        wobble.wobblerDown();

        mecanumDrive.followTrajectory(secondWobble);
        sleep(500);
        wobble.wobblerClose();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(200); //end of the grabbing and dropping of the second wobbler

        mecanumDrive.followTrajectory(linePark);
        launcher.intakeFlipperRun(0);
    }

    public void autoStack1(){

        //2 wobblers, shoots stack

        Trajectory dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())//drops first wobbler
                .splineToSplineHeading(new Pose2d(70, 2, 0), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(83, -8, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Trajectory wobbleLeave = mecanumDrive.trajectoryBuilder(dropWobbler.end())//so we dont hit wobbler
                .strafeRight    (10)
                .build();
        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(wobbleLeave.end(), true)//goes to shoot
                .splineToSplineHeading(new Pose2d(37, -12, Math.toRadians(-6)), Math.toRadians(180))
                .build();


        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(//slows down
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL/2, TRACK_WIDTH)));

        Trajectory ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end())//grabs ring
                .splineToConstantHeading(new Vector2d(28, -16), Math.toRadians(180))
                .build();

        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(//speeds back up
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)));


        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(ringPickUp.end())//shoots the ring it grabbed
                .splineToLinearHeading(new Pose2d(37, -12, Math.toRadians(-6)), Math.toRadians(0))
                .build();


        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition2.end())//grabs second wobbler
                .splineToSplineHeading(new Pose2d(28, -19, Math.toRadians(90)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(23.3, -19, Math.toRadians(90)), Math.toRadians(180))
                .build();

        Trajectory dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())//drops the second wobbler
                .splineToLinearHeading(new Pose2d(74, -8, Math.toRadians(-90)), Math.toRadians(180))
                .build();

        Trajectory linePark = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end(), true)//parks on the line
                .strafeRight(5)
                .splineToSplineHeading(new Pose2d(65, -30, 0), Math.toRadians(180))
                .build();


        RobotLog.dd(tag,"init done");
        wobble.wobblerClose();
        wobble.wobblerFull();

        while (!isStarted() && !isStopRequested()) {//waits for it to start
            telemetry.addData("stack ", stack);
            telemetry.update();
            RobotLog.dd(tag, "waitforstart");
        }


        RobotLog.dd(tag,"start");


        launcher.SpinFlywheel(1);

        wobble.wobblerUp();
        mecanumDrive.followTrajectory(dropWobbler);
        wobble.wobblerDown();
        sleep(100);
        wobble.wobblerOpen();

        mecanumDrive.followTrajectory(wobbleLeave);

        RobotLog.dd(tag,"wobble1done"); //marks the end of dropping the first wobbler
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
        RobotLog.dd(tag,"shoot1done");//marks the end of shooting the starting rings


        launcher.inTake.setPower(1);
        launcher.inTake2.setPower(1);
        sleep(100);
        mecanumDrive.followTrajectory(ringPickUp);
        sleep(1400);
        launcher.inTake.setPower(0);
        launcher.inTake2.setPower(0);
        launcher.MovePusher(0);
        mecanumDrive.followTrajectory(shootPosition2);
        launcher.MovePusher(0.2);
        sleep(700);
        launcher.MovePusher(0);
        RobotLog.dd(tag,"shoot2done");//marks end of shooting starter stack


        wobble.wobblerDown();

        mecanumDrive.followTrajectory(secondWobble);
        sleep(500);
        wobble.wobblerClose();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(200);
        RobotLog.dd(tag,"wobble2done"); //marks end of grabbing and dropping second wobbler

        mecanumDrive.followTrajectory(linePark);
        launcher.intakeFlipperRun(0);
        RobotLog.dd(tag,"alldone"); //marks parking on line
    }

    public void autoStack4() {

        //does two wobblers, and shoots 3 rings in starter stack

        Trajectory dropWobbler = mecanumDrive.trajectoryBuilder(new Pose2d())//drops wobbler one
                .splineToLinearHeading(new Pose2d(120, 0, 0), Math.toRadians(0))
                .build();
        RobotLog.dd(tag,"wobblerdrop1init");

        Trajectory wobbleLeave = mecanumDrive.trajectoryBuilder(dropWobbler.end())//moves away from wobbler
                .strafeRight(10)
                .build();

        RobotLog.dd(tag,"wobblerleaveinit");

        Trajectory shootPosition1 = mecanumDrive.trajectoryBuilder(wobbleLeave.end(), true)//shoots starting rings
                .splineToSplineHeading(new Pose2d(37, -27, Math.toRadians(5)), Math.toRadians(180))
                .build();

        RobotLog.dd(tag,"shoot1init");

        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(//slow down
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL/2, TRACK_WIDTH)));
        RobotLog.dd(tag,"velconstraintinit");
        Trajectory ringPickUp = mecanumDrive.trajectoryBuilder(shootPosition1.end().plus(new Pose2d(0,0, Math.toRadians(-110))))//grabs 3 rings
                .lineTo(new Vector2d(36, -8))
                .build();
        RobotLog.dd(tag,"ringpickupinit");
        mecanumDrive.velConstraint = new MinVelocityConstraint(Arrays.asList(//speed up
                new AngularVelocityConstraint(MAX_ANG_VEL),
                new MecanumVelocityConstraint(MAX_VEL, TRACK_WIDTH)));

        RobotLog.dd(tag,"velconstraint2init");
        Trajectory shootPosition2 = mecanumDrive.trajectoryBuilder(ringPickUp.end())//shoots starter stack rings
                .splineToLinearHeading(new Pose2d(38, -8, Math.toRadians(10)), Math.toRadians(0))
                .build();
        RobotLog.dd(tag,"shoot2init");

        Trajectory secondWobble = mecanumDrive.trajectoryBuilder(shootPosition2.end())//grabs second wobbler
                .splineToSplineHeading(new Pose2d(28, -18, Math.toRadians(90)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(25, -18, Math.toRadians(90)), Math.toRadians(180))
                .build();
        RobotLog.dd(tag,"wobbler2init");
        Trajectory dropSecondWobbler = mecanumDrive.trajectoryBuilder(secondWobble.end())//drops second wobbler
                .splineToLinearHeading(new Pose2d(113, -6, Math.toRadians(-10)), Math.toRadians(45))
                .build();
        RobotLog.dd(tag,"wobblerdrop2init");
        Trajectory linePark = mecanumDrive.trajectoryBuilder(dropSecondWobbler.end(), true)//parks on the line
                .strafeRight(5)
                .splineToSplineHeading(new Pose2d(65, -30, 0), Math.toRadians(180))
                .build();
        RobotLog.dd(tag,"lineparkinit");

        RobotLog.dd(tag,"init done");
        wobble.wobblerClose();
        wobble.wobblerFull();

        while (!isStarted() && !isStopRequested()) {//waits for start
            telemetry.addData("stack ", stack);
            telemetry.update();
//            RobotLog.dd(tag, "waitforstart");
        }


        RobotLog.dd(tag,"start");


        launcher.SpinFlywheel(1);

        wobble.wobblerUp();
        mecanumDrive.followTrajectory(dropWobbler);
        wobble.wobblerDown();
        sleep(100);
        wobble.wobblerOpen();

        mecanumDrive.followTrajectory(wobbleLeave);

        RobotLog.dd(tag,"wobble1done");//marks wobbler one being dropped
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
        RobotLog.dd(tag,"shoot1done");//marks the starter rings being shot


        mecanumDrive.turn(Math.toRadians(-110 - mecanumDrive.getPoseEstimate().getHeading()));
        launcher.inTake.setPower(1);
        launcher.inTake2.setPower(1);
        launcher.MovePusher(0);
        sleep(100);
        mecanumDrive.followTrajectory(ringPickUp);
        sleep(700);
        mecanumDrive.followTrajectory(shootPosition2);
        launcher.inTake.setPower(0);
        launcher.inTake2.setPower(0);
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

        RobotLog.dd(tag,"shoot2done");//marks intaking and shooting of starter stack rings

        wobble.wobblerDown();

        mecanumDrive.followTrajectory(secondWobble);
        sleep(300);
        wobble.wobblerClose();
        sleep(500);

        mecanumDrive.followTrajectory(dropSecondWobbler);
        wobble.wobblerDown();
        wobble.wobblerOpen();
        sleep(200);
        RobotLog.dd(tag,"wobble2done");//marks grabbing and dropping of second wobbler

        mecanumDrive.followTrajectory(linePark);
        launcher.intakeFlipperRun(0);
        RobotLog.dd(tag,"alldone");//marks parking on the line

    }
}