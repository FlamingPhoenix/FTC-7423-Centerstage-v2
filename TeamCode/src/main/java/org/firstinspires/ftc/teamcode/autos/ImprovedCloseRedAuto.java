package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@TeleOp(name = "Improved Close Red Auto", group = "Autos")
public class ImprovedCloseRedAuto extends LinearOpMode {
    final double[][] spikePlace = {{36.00,26.00},{24.00,26.00},{36.00,14}};
    final String[] propPlaceNames = {"left","center","right"};
    @Override
    public void runOpMode() throws InterruptedException {
        // DETECTIOM
        OpenCVPropDetector detector = new OpenCVPropDetector(hardwareMap);
        int detection = detector.detectRed().getValue();
        telemetry.addData("Detection", propPlaceNames[detection]);
        //ROADRRUNNER
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(60,12,Math.toRadians(90)));


        TrajectorySequence main = drive.trajectorySequenceBuilder(new Pose2d(60, 12.00, Math.toRadians(90.00)))
                .splineToConstantHeading(new Vector2d(36.00, 50.00), Math.toRadians(90.00))
                .addTemporalMarker(()->{
                    System.out.println("Marker 1");
                })
                .waitSeconds(1)
                .lineToConstantHeading(new Vector2d(spikePlace[detection][0], spikePlace[detection][1]))
                .addTemporalMarker(()->{
                    System.out.println("Marker 2");
                })
                .waitSeconds(1)

                .splineToConstantHeading(new Vector2d(60.00, -0.00), Math.toRadians(270.00))
                .lineTo(new Vector2d(60.00, -24.00))
                .splineToConstantHeading(new Vector2d(36.00, -50.00),Math.toRadians(270))
                .waitSeconds(1)
                .addTemporalMarker(() -> {

                })
                .splineToConstantHeading(new Vector2d(60, -24),Math.toRadians(90))
                .lineTo(new Vector2d(60,0))
                .splineToConstantHeading(new Vector2d(40.00, 50.00), Math.toRadians(90))
                        .build();
        waitForStart();
    }
}
