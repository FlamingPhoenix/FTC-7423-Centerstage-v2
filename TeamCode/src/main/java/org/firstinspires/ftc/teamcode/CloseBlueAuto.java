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

@Autonomous
public class CloseBlueAuto extends LinearOpMode {


    PropDetectionBlue propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 1";

    Claw claw;
    AxonServo armServo;
    LinkageArm arm;
    ServoDegreeController wrist;
    ServoStates servoController;
    final double armFloor = 0.005;
    final double wristFloor = 0.347;
    final double[] xpossR = {32.96, 26.96,20.96};
    final int[] rotsR = {90, 0, 270};
    final double[] spikePlaceX = {0, 2.5, -1.2};
    final double[] spikePlaceY = {4, 6, -3};



    @Override
    public void runOpMode() {

        claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.43, 0.562,0.562, true);//TODO set positions
        arm = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        armServo = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5);//TODO: set max and min (or zero pos)
        servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

        servoController.addState("transfer",new double[]{0.88,0.567,-1,0.035});
        servoController.addState("transferIntake",new double[]{0.88,0.889,-1,0.035});

        servoController.addState("transferInter",new double[]{0.836,0.62777,-1,0.10});

        servoController.addState("intermediate",new double[]{0.836,-1,0,0.4});
        servoController.addState("low",new double[]{0.31733,0.41333,-1,0.7});
        servoController.addState("high",new double[]{1,0.748,-1,0.62});
        servoController.addState("intake",new double[]{0.300,0.29,0.562,0.036});

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionBlue();
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
            telemetry.addData("Blue Amount 1: ", propDetectionRed.getRedAmount1());
            telemetry.addData("Blue Amount 2: ", propDetectionRed.getRedAmount2());
            telemetry.addData("Blue Amount 3: ", propDetectionRed.getRedAmount3());


            telemetry.update();
        }

        waitForStart();

        PlacementPosition placementPosition = propDetectionRed.getPlacementPosition();

        // Notify subsystems before loop
        int detection = 1;
        switch (placementPosition) {
            case RIGHT:
                telemetry.addLine("RIGHT");
                detection = 2;
                break;
            case LEFT:
                telemetry.addLine("LEFT");
                detection = 0;
                break;
            case CENTER:
                telemetry.addLine("CENTER");
                detection = 1;
                break;
        }
        telemetry.update();
        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .lineToSplineHeading(new Pose2d(xpossR[detection], -32.50, Math.toRadians(270.00))) //ADJUST Y TO YOUR NEEDS (FROM ARMBACKDROP position)
                .addTemporalMarker(0.5,() -> {
                    servoController.setState("intermediate");
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    servoController.setState("high");
                })
                .waitSeconds(2)
                .addTemporalMarker(()->{
                    claw.halfOpen();
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    claw.setPosition(0f);
                    servoController.setState("transferIntake");
                })
                .waitSeconds(2)
                .lineToLinearHeading(new Pose2d(30.50 + spikePlaceX[detection], 12.00 + spikePlaceY[detection], Math.toRadians(rotsR[detection])))//PLACE PURPLE: claw on ground, full open, transfer
                .addTemporalMarker(() -> {
                    armServo.setPosition(armFloor);
                    wrist.setPosition(wristFloor);
                    claw.open();
                })
                .waitSeconds(1)
                .addTemporalMarker(()->{
                    claw.close();
                    servoController.setState("transferIntake");
                })
                .lineToSplineHeading(new Pose2d(63.00, -31.00, Math.toRadians(270.00)))
                .build();
        drive.followTrajectorySequence(ts);

    }
}