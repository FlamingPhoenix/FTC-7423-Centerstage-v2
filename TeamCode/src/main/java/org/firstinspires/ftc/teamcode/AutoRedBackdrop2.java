package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.vision.VisionPortal;

public class AutoRedBackdrop2 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";

    final double[] xpossB = {-40.96,-34.96,-28.96,-34.96};
    final double[] xpossR = {28.96, 34.96, 40.96, 34.96};
    final int[] rotsR = {270, 0, 90};
    final int[] rotsB = {90,180,-90};

    final double armBackdrop = 0.0000000000;//TODO: GET THESE VALUES!!!!!!!
    final double wristBackdrop = 0.0000000000;
    final double armFloor = 0.005;
    final double wristFloor = 0.8;
    VisionPortal visionPortal;


    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap.servo.get("claw"), 0, 0, 0, 0, true);//GET POSITIONS
        AxonServo arm = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        ServoDegreeController wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5); // SET ZERO POSITION
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        ServoStates servoController = new ServoStates(new Servo[]{hardwareMap.servo.get("linkage"),hardwareMap.servo.get("wrist"),hardwareMap.servo.get("claw"),hardwareMap.servo.get("armservo")});

        servoController.addState("transfer",new double[]{0.836,0.62777,-1,0.035});

        int detection = TFOD.getPos();
        Pose2d startPose = new Pose2d(-60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        waitForStart();
        if(isStopRequested()) return;

        TrajectorySequence ts = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(xpossR[detection], 48.00), Math.toRadians(90.00))
                .addDisplacementMarker(() -> {
// PIXEL YELLOW ON BACKDROP, use ppp, half open, close, transfer
                })
                .lineToLinearHeading(new Pose2d(36.00, 12.00, Math.toRadians(rotsR[detection])))//PLACE PURPLE: claw on ground, full open, transfer
                .addDisplacementMarker(() -> {
                    arm.setPosition(armFloor);
                    wrist.setPosition(wristFloor);
                    claw.halfOpen();
                })
                .waitSeconds(1)
                .addDisplacementMarker(()->{
                    claw.close();
                })
                .lineToSplineHeading(new Pose2d(60.00, 50.00, Math.toRadians(90.00)))
                .build();
    }

}
