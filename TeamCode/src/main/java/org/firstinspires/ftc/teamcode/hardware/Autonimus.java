//package org.firstinspires.ftc.teamcode.hardware;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
////import org.firstinspires.ftc.teamcode.AutoMethods;
//import org.firstinspires.ftc.teamcode.vision.UltamiteGoalPipeline;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvWebcam;
//
////STUFF IS COMMENTED FOR TESTING, UNCOMMENT BEFORE USE
//
//@Autonomous(name="AUTO", group="Linear Opmode")
//@Disabled
//public class Autonimus extends AutoMethods {
//
//    public void runOpMode() {
//
//        mecanumDrive = new MecanumDrive(hardwareMap);
//        wobble = new Wobbler(hardwareMap, telemetry);
//        launcher = new Launcher(hardwareMap, telemetry);
//        boolean pwrShots = false; //true = go for pwr shots, false = go for high goal
//
//        wobble.wobblerLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "camera"), cameraMonitorViewId);
//        UltamiteGoalPipeline testPipeline = new UltamiteGoalPipeline();
//        webcam.setPipeline(testPipeline);
//        webcam.openCameraDevice();
//        webcam.startStreaming(320,240);
//        wobble.wobblerClose();
//        wobble.wobblerFull();
//        while (!isStarted()) {
//            telemetry.addData("stack", testPipeline.stack);
//            telemetry.update();
//        }
//
//
//        int x = testPipeline.stack;
//        if (x == 4) {
//            x = 2;
//        }
//        webcam.stopStreaming();
//        webcam.closeCameraDevice();
//
////        launcher.elevator.setTargetPosition(-100);
////        launcher.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////        launcher.elevator.setPower(0.3);
//
//
//        wobble.wobblerDown();
//        wobble.wobblerLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        sleep(500);
//        if(!pwrShots){
//            if(x == 0)
//            {
//                FLSquareWobbler();
//            }
//            else if(x == 1)
//            {
//                RSquareWobbler();
//            }
//            else if(x == 2)
//            {
//                BLSquareWobbler();
//            }
//            sleep(5000);
//            for (int i = 0; i < 3; i++) {
//                launcher.MovePusher(0.2);
//                sleep(500);
//                launcher.MovePusher(0);
//                sleep(2000);
//            }
//            goStraight(0.8, 22, 60);
//        } else if (pwrShots){
//            if(x == 0)
//            {
//                FLSquareWobblerPwrShot();
//            }
//            else if(x == 1)
//            {
//                RSquareWobblerPwrShot();
//            }
//            else if(x == 2)
//            {
//                BLSquareWobblerPwrShot();
//            }
//            sleep(5000);
//            for (int i = 0; i < 3; i++) {
//                launcher.MovePusher(0.2);
//                sleep(1000);
//                launcher.MovePusher(0);
//                sleep(1000);
//                goSideways(0.6, 7, 60);
//            }
//            goStraight(0.7, 20, 60);
//
////            launcher.elevator.setTargetPosition(0);
////            launcher.elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////            launcher.elevator.setPower(-0.3);
//            sleep(500);
//            telemetry.addLine("Elevator " + launcher.elevator.getCurrentPosition());
//            telemetry.update();
//
//        }
////test
//    }
//}
