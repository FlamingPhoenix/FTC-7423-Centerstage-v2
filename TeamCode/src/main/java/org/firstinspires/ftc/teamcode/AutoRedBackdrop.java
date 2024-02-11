package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class AutoRedBackdrop extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";

    //final double[] xpossB = {-40.96,-34.96,-28.96,-34.96};
    final double[] xpossR = {28.96, 34.96, 40.96, 34.96};
    final int[] rotsR = {270, 180, 90};

    final double armBackdrop = 0.0000000000;//TODO: GET THESE VALUES!!!!!!!
    final double wristBackdrop = 0.0000000000;
    final double armFloor = 0.005;
    final double wristFloor = 0.8;
    VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap.servo.get("claw"), 0, 0, 0, 0, true);//GET POSITIONS
        AxonServo arm = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        LinkageArm linkage = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        ServoDegreeController wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5); // SET ZERO POSITION
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DistanceSensor fcDistanceSensor = hardwareMap.get(DistanceSensor.class, "fc");
        TFOD.initTfod(hardwareMap.get(WebcamName.class, "Webcam 1"), visionPortal, TFOD_MODEL_ASSET);
        PerfectPixelPlacement ppp = new PerfectPixelPlacement(linkage, arm, wrist, fcDistanceSensor);
        ServoStates servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"), hardwareMap.servo.get("wrist"), hardwareMap.servo.get("claw"), hardwareMap.servo.get("armservo")});
        servoController.addState("transfer", new double[]{0.62, 0.5, 0, 0.135});
        servoController.addState("low", new double[]{0.22333, 0.41333, 0.10166, 0.7});
        servoController.addState("high", new double[]{0.22222, 0.2655, 0.1277, 0.595});
        servoController.addState("intake", new double[]{0.303, 0.803, 0.176, 0.033});
        int detection = TFOD.getPos();
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        //PerfectPixelPlacement ppp = new PerfectPixelPlacement(172,arm,wrist);
        TrajectorySequence go2backdrop = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(xpossR[detection], 48.00), Math.toRadians(90.00))//TODO: PERFECT y
                .build();
        TrajectorySequence go2spike = drive.trajectorySequenceBuilder(go2backdrop.end())
                .lineToLinearHeading(new Pose2d(36.00, 12.00, Math.toRadians(rotsR[detection])))
                .build();
        TrajectorySequence park = drive.trajectorySequenceBuilder(go2spike.end())
                .lineToSplineHeading(new Pose2d(60.00, 50.00, Math.toRadians(90.00))) // PARK TOWARDS RED
                .build();
        //GO TO SPIKE MARKS, PLACE PURPLE PIXEL
        //arm.setPosition([floor position])
        //wrist.setPosition([floor angle])
        //TrajectorySequence go2stack = drive.trajectorySequenceBuilder(go2backdrop.end())
        //        .lineToConstantHeading(new Vector2d(36.00, -54.00))
        //        .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(go2backdrop);
        //arm.setPos(armBackdrop);
        //wrist.setPosition(wristBackdrop);
        double distance = fcDistanceSensor.getDistance(DistanceUnit.MM);
        ppp.executeForAuto(distance, 400);
//        servoController.setState("low");

        sleep(3000);

        claw.halfOpen();

        sleep(1000);


        claw.close();
        arm.setPosition(armFloor);
        wrist.setPosition(wristFloor);

        sleep(500);

        drive.followTrajectorySequence(go2spike);

        sleep(1000);

        claw.open();

        sleep(2000);
        servoController.setState("transfer");
        drive.followTrajectorySequence(park);
    }

}