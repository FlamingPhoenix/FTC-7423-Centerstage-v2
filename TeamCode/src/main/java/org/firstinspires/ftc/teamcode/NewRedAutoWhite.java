package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class NewRedAutoWhite extends LinearOpMode {


    PropDetectionRed propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    ServoDegreeController wrist;
    ServoStates servoController;
    final double armFloor = 0.005;
    final double wristFloor = 0.347;
    final double[] xpossR = {35.96, 29.96, 23.96};
    final int[] rotsR = {90, 80, 90};
    final double[] spikePlaceX = {0, -1.2, 1.2};
    final double[] spikePlaceY = {3, 9, 29};
    @Override
    public void runOpMode() throws InterruptedException{

        claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.1234, 0.362,0.362, true);//TODO set positions
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

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(12, -65, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        claw.setPosition(0f);
        servoController.setState("transferIntake");
        TrajectorySequence tsMid = drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(90)))
                .lineTo(new Vector2d(12,-43))
                //purple pixel
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(44,-35,Math.toRadians(180)))
                //yellow pixel
                .waitSeconds(1)
                //first cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .setReversed(false)
                //second cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(false)
                .build();
        TrajectorySequence tsLeft = drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(90)))
                .lineTo(new Vector2d(12,-43))
                //purple pixel
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(44,-35,Math.toRadians(180)))
                //yellow pixel
                .waitSeconds(1)
                //first cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .setReversed(false)
                //second cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(false)
                .build();
        TrajectorySequence tsRight = drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(90)))
                .lineTo(new Vector2d(12,-43))
                //purple pixel
                .waitSeconds(1)
                .lineToSplineHeading(new Pose2d(44,-35,Math.toRadians(180)))
                //yellow pixel
                .waitSeconds(1)
                //first cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .setReversed(false)
                //second cycle
                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                .lineTo(new Vector2d(-8.95,-60))
                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                .waitSeconds(1)
                .setReversed(false)
                .build();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRed();
        camera.setPipeline(propDetectionRed);

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


        while (!isStarted()) {
            telemetry.addData("ROTATION: ", propDetectionRed.getPlacementPosition());
            telemetry.addData("Red Amount 1: ", propDetectionRed.getRedAmount1());
            telemetry.addData("Red Amount 2: ", propDetectionRed.getRedAmount2());
            telemetry.addData("Red Amount 3: ", propDetectionRed.getRedAmount3());


            telemetry.update();
        }

        waitForStart();

        PlacementPosition placementPosition = propDetectionRed.getPlacementPosition();
        int detection = 1;
        switch (placementPosition) {
            case RIGHT:
                telemetry.addLine("RIGHT");
                drive.followTrajectorySequence(tsRight);
                break;
            case LEFT:
                telemetry.addLine("LEFT");
                drive.followTrajectorySequence(tsLeft);

                break;
            case CENTER:
                telemetry.addLine("CENTER");
                drive.followTrajectorySequence(tsMid);
                break;
        }
        telemetry.update();

    }
}