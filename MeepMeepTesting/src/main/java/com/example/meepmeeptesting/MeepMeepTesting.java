package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(40, -50, Math.toRadians(180)))
   //                             .forward(30)
   //                             .turn(Math.toRadians(-90))
   //                             .forward(30)
    //                            .turn(Math.toRadians(-90))
    //                            .forward(30)
    //                            .turn(Math.toRadians(-90))
    //                            .forward(30)
    //                            .turn(Math.toRadians(-90))
                                .splineTo(new Vector2d(-34,-42),Math.toRadians(90) )
                                .splineTo(new Vector2d(-34,50),0)
                                .splineTo(new Vector2d(28,45),Math.toRadians(270) )
                                .splineTo(new Vector2d( 40,-50),Math.toRadians(180))

                                .build()
                );

        Image img = null;
        try { img = ImageIO.read(new File("/Users/dgirard/StudioProjects/road-runner-quickstart-2024/meepmeep.png")); }
        catch (IOException e) {}

/*        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
*/
        meepMeep.setBackground(MeepMeep.Background.)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}