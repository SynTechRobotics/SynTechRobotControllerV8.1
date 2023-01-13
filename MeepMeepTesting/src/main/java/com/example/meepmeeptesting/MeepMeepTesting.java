package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(39.67818276432781, 30, Math.toRadians(60), Math.toRadians(60), 14.5)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-60, -36, 0))
                                .forward(4)
                                .turn(Math.toRadians(-90))
                                .strafeLeft(40)
                                .forward(2)
                                .back(2)
                                .strafeLeft(25)
                                .strafeRight(12)
                                .forward(22.75)
                                .back(22.75)
                                .strafeRight(14)
                                .forward(2)
                                .back(2)
                                .strafeLeft(13.5)
                                .forward(22.5)
                                .back(2)
                                .strafeLeft(13.5)
                                .forward(21)
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}