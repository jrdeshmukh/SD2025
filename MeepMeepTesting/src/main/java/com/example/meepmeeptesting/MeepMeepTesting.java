package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double pi = Math.PI;


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4554535190)
                .setStartPose(new Pose2d(-28, -61, Math.PI/2))
                .build();


        Action pix1 = myBot.getDrive().actionBuilder(myBot.getPose())
                .splineToConstantHeading(new Vector2d(-47.4, -38.5), pi/2)
                .lineToY(-37)
                .waitSeconds(2)
                .lineToY(-40)

                .strafeToLinearHeading(new Vector2d(-50.1, -50.1), pi*5/4)
                .strafeToConstantHeading(new Vector2d(-54.1, -54.1))
                .strafeToConstantHeading(new Vector2d(-50.1, -50.1))

                .splineToLinearHeading(new Pose2d(-58.5, -40, pi/2), Math.PI/2)
                .waitSeconds(2)

                .strafeToLinearHeading(new Vector2d(-50.1, -50.1), pi*5/4)
                .strafeToConstantHeading(new Vector2d(-54.1, -54.1))
                .strafeToConstantHeading(new Vector2d(-50.1, -50.1))

                .strafeToLinearHeading(new Vector2d(-57.6, -24), pi)

                .strafeToLinearHeading(new Vector2d(-50.1, -50.1), pi*5/4)
                .strafeToConstantHeading(new Vector2d(-54.1, -54.1))
                .strafeToConstantHeading(new Vector2d(-50.1, -50.1))
                .build();

        Action goBasket = myBot.getDrive().actionBuilder(new Pose2d(-47.4, -40, pi/2))

                .build();

        Action pix2 = myBot.getDrive().actionBuilder(new Pose2d(-50.1, -50.1, Math.PI*5/4))
                .build();

        myBot.runAction(new SequentialAction(
                pix1
        ));

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}