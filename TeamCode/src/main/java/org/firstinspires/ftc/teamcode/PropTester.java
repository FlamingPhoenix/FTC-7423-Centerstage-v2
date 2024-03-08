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
public class PropTester extends LinearOpMode {


    PropDetectionRedC270 propDetectionRed;
    OpenCvCamera camera;
    String webcamName = "Webcam 2";

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


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        propDetectionRed = new PropDetectionRedC270();
        camera.setPipeline(propDetectionRed);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
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
                break;
            case LEFT:
                telemetry.addLine("LEFT");

                break;
            case CENTER:
                telemetry.addLine("CENTER");
                break;
        }
        telemetry.update();

    }
}