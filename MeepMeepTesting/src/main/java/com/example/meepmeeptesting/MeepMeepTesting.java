package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 11)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-6, -32, Math.toRadians(90)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-57,-45, Math.toRadians(90)),Math.toRadians(180))

                .build());
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-57, -46, Math.toRadians(90)))
                        .setReversed(true)
                        .turn(Math.toRadians(225))
                                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}