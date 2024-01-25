package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utility.AxonServo;
import org.firstinspires.ftc.teamcode.utility.Claw;
import org.firstinspires.ftc.teamcode.utility.LinkageArm;
import org.firstinspires.ftc.teamcode.utility.ServoDegreeController;
import org.firstinspires.ftc.vision.VisionPortal;

@com.qualcomm.robotcore.eventloop.opmode.Autonomous
public class AutoRB extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "/sdcard/FIRST/tflitemodels/model.tflite";
    VisionPortal visionPortal;
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap.servo.get("claw"),0,0,0,true);
        AxonServo arm = new AxonServo(hardwareMap.servo.get("armservo"), hardwareMap.analogInput.get("axonin"));
        ServoDegreeController wrist = new ServoDegreeController(hardwareMap.servo.get("wrist"), 300, 0.5); // SET ZERO POSITION
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        TFOD.initTfod(hardwareMap.get(WebcamName.class,"Webcam 1"),visionPortal, TFOD_MODEL_ASSET);
        int detection = TFOD.getPos();
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        TrajectorySequence go2backdrop = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(36.00, 48.00), Math.toRadians(90.00))
                .build();
        //GO TO SPIKE MARKS, PLACE PURPLE PIXEL
        //arm.setPosition([floor position])
        //wrist.setPosition([floor angle])
        TrajectorySequence go2stack = drive.trajectorySequenceBuilder(go2backdrop.end())
                .lineToConstantHeading(new Vector2d(36.00, -54.00))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(go2backdrop);
    }

}
