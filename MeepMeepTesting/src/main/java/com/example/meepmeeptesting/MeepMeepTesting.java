package com.example.meepmeeptesting;

//IMPORTS
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.ColorScheme;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedLight;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import org.jetbrains.annotations.NotNull;
import java.awt.Color;

// TODO:
// IMPORTANT: READ THIS:
// MEEPMEEP FOR ROADRUNNER V1.0 IS STILL IN BETA,
// MEANING THAT TRAJECTORIES THAT YOU ASK TO RUN
// MAY CONTAIN ERRORS AND NOT BE ACCURATE TO THE
// REAL TRAJECTORIES. THIS EXAMPLE CODE IS ONLY
// MEANT TO BE A BASE AND GUIDELINE FOR BUILDING
// TRAJECTORIES IN THE FUTURE.


public class MeepMeepTesting {
    public static void main(String[] args) {
        //sets window size. Lower number if window is too large
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity ExampleBot = new DefaultBotBuilder(meepMeep)
                // Dimensions: robot size
                .setDimensions(14,14)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 9.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(12, -65, Math.toRadians(90)))
                                .lineTo(new Vector2d(12,-43))
                                .waitSeconds(1)
                                .lineToSplineHeading(new Pose2d(44,-31,Math.toRadians(180)))
                                .waitSeconds(1)
                                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                                .waitSeconds(1)
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                                .lineTo(new Vector2d(-8.95,-60))
                                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                                .setReversed(false)
                                //second cycle
                                .splineTo(new Vector2d(8.95, -60), Math.toRadians(178.00))
                                .splineTo(new Vector2d(-24,-59),Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-58.02, -36), Math.toRadians(92.07))
                                .waitSeconds(1)
                                .setReversed(true)
                                .lineToSplineHeading(new Pose2d(-28,-59,Math.toRadians(180)))
                                .lineTo(new Vector2d(-8.95,-60))
                                .splineToConstantHeading(new Vector2d(44,-35),Math.toRadians(92.07))
                                .waitSeconds(1)
                                .setReversed(false)
                                .build()
                );
        //Declare trajectory: STARTING POSE STARTING DIRECTION

        //sets field
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                //next two lines have no uses that I know of
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                //adds the robot to the field
                .addEntity(ExampleBot)
                //starts code
                .start();
    }
}