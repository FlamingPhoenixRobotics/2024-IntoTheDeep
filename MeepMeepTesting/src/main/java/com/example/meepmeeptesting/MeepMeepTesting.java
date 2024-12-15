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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-12, -62, Math.toRadians(90)))
                .splineToSplineHeading(new Pose2d(-6,-33, Math.toRadians(90)), Math.toRadians(75))

                .setTangent(Math.toRadians(200))
                                .setReversed(true)
                                .splineTo(new Vector2d(-52,-52),Math.toRadians(180))
                                //.lineToXLinearHeading(-53, Math.toRadians(225))
                //.turn(Math.toRadians(135))
                //.setReversed(true)
                //.splineToLinearHeading(new Pose2d(-53,-53, Math.toRadians(-135)), Math.toRadians(180))

                //.splineToLinearHeading(new Pose2d(-53,-53, Math.toRadians(-135)), Math.toRadians(90))


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}