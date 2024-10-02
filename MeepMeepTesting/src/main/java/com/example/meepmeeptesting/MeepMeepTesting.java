//from here; https://github.com/acmerobotics/MeepMeep?tab=readme-ov-file#-installing-android-studio




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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36, -62, Math.toRadians(90)))
//go get the first red block
                .splineToConstantHeading(new Vector2d(42,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
                 .splineToConstantHeading(new Vector2d (52,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d( 62,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
                .lineToY(-54)
 //Now let's go place the clip
                .splineToConstantHeading(new Vector2d(10,-34), Math.toRadians(180))

               .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}