package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
import com.acmerobotics.roadrunner.geometry.Vector2d;
@Autonomous
public class CloseRedAutoWhite extends LinearOpMode {


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

        claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.411, 0.592,0.362, true);
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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRed();
        camera.setPipeline(propDetectionRed);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        claw.setPosition(0f);
        servoController.setState("transferIntake");
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
                detection = 0;
                break;
            case LEFT:
                telemetry.addLine("LEFT");
                detection = 2;
                break;
            case CENTER:
                telemetry.addLine("CENTER");
                break;
        }
        Pose2d firstEnd = new Pose2d(20, 54-spikePlaceY[detection], Math.toRadians(rotsR[detection]));
        telemetry.update();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(xpossR[detection], 56.25, Math.toRadians(90.00))) //ADJUST Y TO YOUR NEEDS (FROM ARMBACKDROP position)
                .addTemporalMarker(0.5,() -> {
                    servoController.setState("intermediate");
                })
                .addTemporalMarker(1,() -> {
                    servoController.setState("high");
                })
                .waitSeconds(1f)
                .addTemporalMarker(()->{
                    claw.halfOpen();
                })
                .waitSeconds(1f)
                .addTemporalMarker(()->{
                    claw.close();
                    servoController.setState("transfer");
                })
                .waitSeconds(0.5f)
                .lineToLinearHeading(new Pose2d(20, 54-spikePlaceY[detection], Math.toRadians(rotsR[detection])))//PLACE PURPLE: claw on ground, full open, transfer
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    servoController.setState("intakeNew");
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    claw.setPosition(0.65f);
                })
                .waitSeconds(0.5f)
                .addTemporalMarker(()->{
                    servoController.setState("transferIntake");
                })
                .lineToSplineHeading(new Pose2d(0.44,23.75,Math.toRadians(90)))
                .lineTo(new Vector2d(0,-50.90))
                .lineTo(new Vector2d(0.44,23.75))
                .build();
        TrajectorySequence whitets = drive.trajectorySequenceBuilder(firstEnd)
                .lineToSplineHeading(new Pose2d(0.44,23.75,Math.toRadians(90)))
                .lineTo(new Vector2d(0,-56.90))
                .splineTo(new Vector2d(17.2,48.61), Math.toRadians(-47.73))
                .build();
        drive.followTrajectorySequence(ts);
        //drive.followTrajectorySequence(whitets);
        TrajectorySequence untitled0 = drive.trajectorySequenceBuilder(new Pose2d(12.34, -61.69, Math.toRadians(90.00)))
                .splineTo(new Vector2d(45.48, -36.64), Math.toRadians(35.77))
                .lineToSplineHeading(new Pose2d(23.75, -13.44, Math.toRadians(0.00)))
                .lineTo(new Vector2d(-56.90, -12.00))
                .splineTo(new Vector2d(48.61, -37.20), Math.toRadians(-47.73))
                .build();


    }
}