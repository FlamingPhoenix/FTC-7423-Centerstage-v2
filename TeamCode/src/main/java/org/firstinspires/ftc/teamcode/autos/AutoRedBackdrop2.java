package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PerfectPixelPlacement;
import org.firstinspires.ftc.teamcode.ServoStates;
import org.firstinspires.ftc.teamcode.TFOD;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
public class AutoRedBackdrop2 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";

    final double[] xpossB = {-40.96,-34.96,-28.96,-34.96};
    final double[] xpossR = {28.96, 34.96, 40.96, 34.96};
    final int[] rotsR = {270, 0, 90,0};
    final int[] rotsB = {90,180,-90,0};

    final double armBackdrop = 0.7;//TODO: GET THESE VALUES!!!!!!!
    final double wristBackdrop = 0.2655;
    final double armFloor = 0.005;
    final double wristFloor = 0.8;
    VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap.servo.get("claw"), 0, 0.053, 0.183,0.193, true);//GET POSITIONS
        AxonServo arm = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        ServoDegreeController wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5); // SET ZERO POSITION
        LinkageArm linkage = new LinkageArm(hardwareMap.servo.get("linkage"), 175, 236);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PerfectPixelPlacement ppp = new PerfectPixelPlacement(linkage,arm, wrist, hardwareMap.get(DistanceSensor.class,"fc"));
        ServoStates servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});
        TFOD.initTfod(hardwareMap.get(WebcamName.class,"Webcam 1"),visionPortal,TFOD_MODEL_ASSET);
        servoController.addState("transfer",new double[]{0.836,0.62777,-1,0.037});

        int detection = TFOD.getPos();
        telemetry.addData("prrop",detection);
        telemetry.update();
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        claw.close();
        waitForStart();
        if(isStopRequested()) return;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(xpossR[detection], 46.00), Math.toRadians(90.00)) //ADJUST Y TO YOUR NEEDS (FROM ARMBACKDROP position)
                .addTemporalMarker(() -> {
// PIXEL YELLOW ON BACKDROP,
                    arm.setPosition(armBackdrop);
                    wrist.setPosition(wristBackdrop);
                })
                .waitSeconds(2)
                .addTemporalMarker(() -> {
                    claw.halfOpen();
                })
                .waitSeconds(5)
                .addTemporalMarker(() -> {
                    claw.close();
                })
                .lineToLinearHeading(new Pose2d(36.00, 12.00, Math.toRadians(rotsR[detection])))//PLACE PURPLE: claw on ground, full open, transfer
                .addTemporalMarker(() -> {
                    arm.setPosition(armFloor);
                    wrist.setPosition(wristFloor);
                    claw.open();
                })
                .waitSeconds(3)
                .addTemporalMarker(()->{
                    claw.close();
                })
                .lineToSplineHeading(new Pose2d(63.00, 50.00, Math.toRadians(90.00)))
                .build();
        drive.followTrajectorySequence(ts);
    }

}
