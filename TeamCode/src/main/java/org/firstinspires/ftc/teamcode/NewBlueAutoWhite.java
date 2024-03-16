package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="NewRedAutoWhite", group="Use This")
public class NewBlueAutoWhite extends NewAprilTag {


    PropDetectionBlueC270 propDetectionBlue;
    OpenCvCamera camera;
    String webcamName = "Webcam 2";

    Servo claw;
    AxonServo armServo;
    LinkageArm arm;
    ServoDegreeController wrist;
    ServoStates servoController;


    //    final double armFloor = 0.005;
//    final double wristFloor = 0.347;
//    final double[] xpossR = {35.96, 29.96, 23.96};
//    final int[] rotsR = {90, 80, 90};
//    final double[] spikePlaceX = {0, -1.2, 1.2};
//    final double[] spikePlaceY = {3, 9, 29};
    @Override
    public void runOpMode() throws InterruptedException{
        claw = hardwareMap.servo.get("claw");
        //claw = new Claw(hardwareMap.servo.get("claw"), 0.29, 0.411, 0.592,0.592, true);//TODO set positions
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);//TODO: set max and min (or zero pos)
        servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

        servoController.addState("transfer",new double[]{0.88,0.567,0,0.035});
        servoController.addState("transferIntake",new double[]{0.88,0.85,0,0.035});

        servoController.addState("transferInter",new double[]{0.836,0.62777,-1,0.10});

        servoController.addState("intermediate",new double[]{0.836,-1,0,0.4});
        servoController.addState("low",new double[]{0.31733,0.41333,0,0.7});
        servoController.addState("high",new double[]{1,0.748,0,0.62});

        servoController.addState("intakeNew",new double[]{0.300,0.29,0,0.1});
        servoController.addState("intakeStack",new double[]{0.88,0.2994,0.592,0.096666});
        servoController.addState("scanApril",new double[]{0.88,0.644,0.592,0.096666});

        servoController.addState("closeClaw", new double[]{-1,-1,0.29,-1});
        servoController.addState("halfClaw", new double[]{-1,-1,0.411,-1});
        servoController.addState("openClaw", new double[]{-1,-1,0.592,-1});
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        claw.setPosition(0f);
        servoController.setState("transferIntake");

        initCameras();
        TrajectorySequence preloadsLeftOne = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(270)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12,51,Math.toRadians(300)))
                .addTemporalMarker(0,()->{
                    servoController.setState("intakeNew");
                })
                .addTemporalMarker(0.4f,()->{
                    claw.setPosition(0.411);
                })
                //purple pixel
                .waitSeconds(.5)
                .addDisplacementMarker(()->{
                    servoController.setState("transferIntake");
                })
                .build();
        TrajectorySequence preloadsLeftTwo = drive.trajectorySequenceBuilder(new Pose2d(12, 51, Math.toRadians(300)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(66,25,Math.toRadians(0)))
                .addTemporalMarker(1.2,()->{
                    servoController.setState("high");
                })
                .addDisplacementMarker(()->{
                    claw.setPosition(0.592);
                })
                //yellow pixel
                .waitSeconds(1.5f)
                .addTemporalMarker(()->{
                    servoController.setState("scanApril");
                })
                .build();
        TrajectorySequence preloadsMidOne = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(270)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12,51,Math.toRadians(265)))
                .addTemporalMarker(0,()->{
                    servoController.setState("intakeNew");
                })
                .addTemporalMarker(0.4f,()->{
                    claw.setPosition(0.411);
                })
                .waitSeconds(0.5f)
                .addDisplacementMarker(()->{
                    servoController.setState("transferIntake");
                })
                .build();
        TrajectorySequence preloadsMidTwo = drive.trajectorySequenceBuilder(new Pose2d(12, 51, Math.toRadians(265)))
                .setReversed(true)
                .addTemporalMarker(1.2,()->{
                    servoController.setState("high");
                })
                .lineToSplineHeading(new Pose2d(66,29,Math.toRadians(0)))
                .addTemporalMarker(()->{
                    claw.setPosition(0.592);
                })
                .waitSeconds(1.5f)
                .addTemporalMarker(()->{
                    servoController.setState("scanApril");
                })
                .build();
        TrajectorySequence preloadsRightOne = drive.trajectorySequenceBuilder(new Pose2d(12, 65, Math.toRadians(270)))
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(12,51,Math.toRadians(240)))
                .addTemporalMarker(0,()->{
                    servoController.setState("intakeNew");
                })
                .addTemporalMarker(0.3,()-> {
                    claw.setPosition(0.411);
                })
                //purple pixel
                .waitSeconds(1)
                .addDisplacementMarker(()->{
                    //claw.close();
                    servoController.setState("transferIntake");
                })
                .build();
        TrajectorySequence preloadsRightTwo = drive.trajectorySequenceBuilder(new Pose2d(12, 51, Math.toRadians(240)))
                .setReversed(true)
                .addTemporalMarker(1.2,()->{
                    servoController.setState("high");
                })
                .lineToSplineHeading(new Pose2d(66,37,Math.toRadians(0)))
                .addTemporalMarker(()->{
                    claw.setPosition(0.592);
                })
                //yellow pixel
                .waitSeconds(1.5f)
                .addTemporalMarker(()->{
                    servoController.setState("scanApril");
                })
                .build();

        TrajectorySequence dropoffLeft = drive.trajectorySequenceBuilder(preloadsLeftTwo.end())
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(8.95, -69), Math.toRadians(178.00))
                .splineToConstantHeading(new Vector2d(-24,-69),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40.02, -28), Math.toRadians(92.07))
                .addTemporalMarker(1,()->{
                    servoController.setState("scanApril");
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence dropoffMid = drive.trajectorySequenceBuilder(new Pose2d(66,31,Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(8.95, 69), Math.toRadians(178.00))
                .splineToConstantHeading(new Vector2d(-24,69),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40.02, 28), Math.toRadians(92.07))
                .addTemporalMarker(1,()->{
                    servoController.setState("scanApril");
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence dropoffRight = drive.trajectorySequenceBuilder(new Pose2d(66,-37,Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(8.95, -69), Math.toRadians(178.00))
                .splineToConstantHeading(new Vector2d(-24,-69),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40.02, -28), Math.toRadians(92.07))
                .addTemporalMarker(1,()->{
                    servoController.setState("scanApril");
                })
                .waitSeconds(1)
                .build();
        TrajectorySequence score = drive.trajectorySequenceBuilder(new Pose2d(-40.02,28,Math.toRadians(0)))
                .setReversed(false)
                .addTemporalMarker(()->{
                    servoController.setState("transferIntake");
                })
                .splineToConstantHeading(new Vector2d(-28,59), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-8.95,59),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(59,35),Math.toRadians(92.07))
                .addTemporalMarker(2,() -> {
                    servoController.setState("high");
                })
                .waitSeconds(0.4)
                .addTemporalMarker(()->{
                    claw.setPosition(0.592);
                })

                .build();
        TrajectorySequence intake = drive.trajectorySequenceBuilder(new Pose2d(63,35,Math.toRadians(0)))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(8.95, 69), Math.toRadians(178.00))
                .splineToConstantHeading(new Vector2d(-24,69),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-40.02, 28), Math.toRadians(92.07))
                .addTemporalMarker(1,()->{
                    servoController.setState("intakeStack");
                })
                .waitSeconds(1)
                .build();
        //preloadmid1,2    dropoffmid score intake
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionBlue = new PropDetectionBlueC270();
        camera.setPipeline(propDetectionBlue);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        //initall();
        while (!isStarted()) {
            telemetry.addData("ROTATION: ", propDetectionBlue.getPlacementPosition());
            telemetry.addData("Red Amount 1: ", propDetectionBlue.getRedAmount1());
            telemetry.addData("Red Amount 2: ", propDetectionBlue.getRedAmount2());
            telemetry.addData("Red Amount 3: ", propDetectionBlue.getRedAmount3());


            telemetry.update();
        }
        waitForStart();

        PlacementPosition placementPosition = propDetectionBlue.getPlacementPosition();
        camera.closeCameraDevice();
        //aligner = new AprilTagAligner(hardwareMap,"Webcam 1");
        int detection = 1;
        switch (placementPosition) {
            case RIGHT:
                telemetry.addLine("RIGHT");
                drive.followTrajectorySequence(preloadsRightOne);
                drive.followTrajectorySequence(preloadsRightTwo);
                drive.followTrajectorySequence(dropoffRight);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                //alignBack(6,6);
                //drive.setPoseEstimate(new Pose2d(57,-42,Math.toRadians(0)));
                drive.followTrajectorySequence(intake);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                break;
            case LEFT:
                drive.followTrajectorySequence(preloadsLeftOne);
                drive.followTrajectorySequence(preloadsLeftTwo);
                drive.followTrajectorySequence(dropoffLeft);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                //alignBack(6,6);
                //drive.setPoseEstimate(new Pose2d(57,-42,Math.toRadians(0)));
                drive.followTrajectorySequence(intake);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                break;
            case CENTER:
                telemetry.addLine("CENTER");
                drive.followTrajectorySequence(preloadsMidOne);
                drive.followTrajectorySequence(preloadsMidTwo);
                drive.followTrajectorySequence(dropoffMid);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                //alignBack(6,6);
                //drive.setPoseEstimate(new Pose2d(57,-42,Math.toRadians(0)));
                drive.followTrajectorySequence(intake);
                alignFront(9,6);
                servoController.setState("transferIntake");
                drive.setPoseEstimate(new Pose2d(-59,-36,Math.toRadians(0)));
                sleep(500);
                claw.close();
                drive.followTrajectorySequence(score);
                break;
        }

        //drive.followTrajectorySequence(dropoff);
        telemetry.update();

    }
}
/*                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(270)))
                                .setReversed(true)
                                .lineTo(new Vector2d(12,-43))
                                //purple pixel
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(47,-35,Math.toRadians(0)))
                                //yellow pixel
                                .waitSeconds(1)
                                //first cycle
                                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-28,-59), Math.toRadians(0))
                                .splineTo(new Vector2d(-8.95,-60),Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(47,-35),Math.toRadians(92.07))
                                //second cycle
                                .setReversed(true)
                                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                                .waitSeconds(1)
                                .setReversed(false)
                                .splineToConstantHeading(new Vector2d(-28,-59), Math.toRadians(0))
                                .splineTo(new Vector2d(-8.95,-60),Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(47,-35),Math.toRadians(92.07))
                                .build()*/