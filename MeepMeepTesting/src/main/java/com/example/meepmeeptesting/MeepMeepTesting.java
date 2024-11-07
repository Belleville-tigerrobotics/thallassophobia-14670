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

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(15, -61, Math.toRadians(90)))
//start by prepping to clip the specimine
                .splineToConstantHeading(new Vector2d(2,-30),0)


//go get the first red block

                .splineToConstantHeading(new Vector2d(31,-60),0)

                .splineToConstantHeading(new Vector2d(44,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
                 .splineToConstantHeading(new Vector2d (53,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
                .splineToConstantHeading(new Vector2d( 61,-12),0)
                .setTangent(Math.toRadians(90))
                .lineToY(-58)
                .setTangent(Math.toRadians(90))
  //              .lineToY(-54)
 //sweeping done.  can stay here to park, or next step to go ascend


                .splineToConstantHeading(new Vector2d(-37,-44), Math.toRadians(180))
                .setTangent(Math.toRadians(-90))
                .lineToY(-18)

                .turn(Math.toRadians(-65))

                //now ready to extend the wire



/* this is left side park only
                .splineToConstantHeading(new Vector2d(-24,-55),0)

                .splineToConstantHeading(new Vector2d(38,-58),0)
*/

               .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}