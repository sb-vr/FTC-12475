package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {

        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d startPose = new Pose2d(
                10.46 - 72,
                133.54 - 72,
                Math.toRadians(45)
        );

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(
                        50, 50,
                        Math.toRadians(180),
                        Math.toRadians(180),
                        15
                )
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(startPose)

                                .waitSeconds(2.0)
                                .waitSeconds(3.0)

                                .turn(Math.toRadians(-45))

                                .forward(60)

                                .waitSeconds(1.0)

                                .back(60)

                                .turn(Math.toRadians(-45))

                                .waitSeconds(2.0)
                                .waitSeconds(5.0)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}