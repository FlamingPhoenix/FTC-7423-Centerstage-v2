package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous()
public class RoadRunnerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        TrajectorySequence trajectorySequence = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(30, 40),0)
                .lineTo(new Vector2d(0,0))
                .build();

        waitForStart();

        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajectorySequence);
    }
}
