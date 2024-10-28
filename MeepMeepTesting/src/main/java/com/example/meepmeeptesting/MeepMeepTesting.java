package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double pi = Math.PI;

        Pose2d initialPose = new Pose2d(10, -60.5, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4554535190)
                .setStartPose(initialPose)
                .build();


        // vision here that outputs position
        int startPosition = 1;

        TrajectoryActionBuilder dropSpecimen = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(2)
                .strafeTo(new Vector2d(10, -34))
                .waitSeconds(2);

        TrajectoryActionBuilder pickup1 = dropSpecimen.fresh()
                //.strafeTo(new Vector2d(10, -40))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(38, -26, 0), pi/2)
                .waitSeconds(2);

        TrajectoryActionBuilder drop1 = pickup1.fresh()
                .strafeToLinearHeading(new Vector2d(48, -56), 3*pi/2)
                .strafeTo(new Vector2d(48, -59))
                //.strafeTo(new Vector2d(-48, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                //.strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(2);

        TrajectoryActionBuilder pickup2 = drop1.fresh()
                .strafeTo(new Vector2d(48, -56))
                .strafeToLinearHeading(new Vector2d(58, -37.4), pi/2)
                .waitSeconds(2);

        TrajectoryActionBuilder drop2  = pickup2.fresh()
                 .strafeToLinearHeading(new Vector2d(48, -56), 3*pi/2)
                 .strafeTo(new Vector2d(48, -59))
                 .waitSeconds(0.1);

        TrajectoryActionBuilder score1 = drop2.fresh()
              .strafeToLinearHeading(new Vector2d(8, -38), pi/2)
              .strafeTo(new Vector2d(8, -34))
              .waitSeconds(0.1);

        TrajectoryActionBuilder pickup3 = score1.fresh()
                .strafeToLinearHeading(new Vector2d(48, -56), 3*pi/2)
                .strafeTo(new Vector2d(48, -59))
                .waitSeconds(0.1);

        TrajectoryActionBuilder score2 = pickup3.fresh()
                        .strafeTo(new Vector2d(48, -56))
                                .strafeToLinearHeading(new Vector2d(4, -38), pi/2)
                                        .strafeToLinearHeading(new Vector2d(4, -34), pi/2);

        myBot.runAction(
                new SequentialAction(
                        dropSpecimen.build(),
                        pickup1.build(),
                        drop1.build(),
                        pickup2.build(),
                        drop2.build(),
                        score1.build(),
                        pickup3.build(),
                        score2.build()
                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }}