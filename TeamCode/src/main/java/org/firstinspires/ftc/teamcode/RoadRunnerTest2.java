package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadRunnerTest2 extends LinearOpMode {
    @Override
    public void runOpMode(){
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(60, 12, Math.toRadians(180));
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(new Pose2d(60.00, 12.00, Math.toRadians(180.00)))
                .splineTo(new Vector2d(36.00, 48.00), Math.toRadians(90.00))
                .lineToConstantHeading(new Vector2d(36.00, -54.00))
                .build();
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajectorySequence);
    }
}
