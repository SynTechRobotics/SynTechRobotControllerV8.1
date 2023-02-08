package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, 4.1244445800781255, Math.toRadians(180), 13.03)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-65, 38, 0))
                                .lineToLinearHeading(new Pose2d(-53, 36, Math.toRadians(-90)))
                                .lineToConstantHeading(new Vector2d(-9, 37))
                                .lineToLinearHeading(new Pose2d(-25, 32.5, Math.toRadians(-90)))

                                .lineToLinearHeading(new Pose2d(-13, 42, Math.toRadians(90)))
                                .lineToLinearHeading(new Pose2d(-13, 59.5, Math.toRadians(90)))
                                .back(4)
                                .lineToLinearHeading(new Pose2d(-13, 50, Math.toRadians(195)))
                                .forward(2.5)
                                .build()

//                        drive.trajectorySequenceBuilder(new Pose2d(-65, -38, 0))
//                                .lineToLinearHeading(new Pose2d(-53, -36, Math.toRadians(90)))
//                                .lineToConstantHeading(new Vector2d(-12, -37))
//                                .lineToLinearHeading(new Pose2d(-23, -32, Math.toRadians(90)))
//                                .lineToLinearHeading(new Pose2d(-13, -42, Math.toRadians(-90)))
//                                .lineToLinearHeading(new Pose2d(-14, -58, Math.toRadians(-90)))

//                                .strafeLeft(4)
//                                .forward(4)
//                                .strafeRight(3.5)
//                                .turn(Math.toRadians(-90))
//                                .strafeLeft(40)
//                                .forward(2)
//                                .back(2)
//                                .strafeLeft(25)
//                                .strafeRight(12)
//                                .forward(22.75)
//                                .back(23)
//                                .strafeRight(14)
//                                .forward(2)
//                                .back(2)
//                                .strafeLeft(13.5)
//                                .back(23.5)
//                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}