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

public class JayanNEWSPECUIAUTO {
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
                .strafeTo(new Vector2d(10, -41.3))
                .waitSeconds(0.001);

        TrajectoryActionBuilder push1 = dropSpecimen.fresh()
                .strafeTo(new Vector2d(28.5, -41.7))
                .splineToConstantHeading(new Vector2d(46.5, -12), 0)
                //.strafeTo(new Vector2d(47.5, -12))
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(47.5, -54), 3*pi/2)
                .strafeToLinearHeading(new Vector2d(59.6, -37.3), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder spline1 = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(26.5, -45))
                .splineToConstantHeading(new Vector2d(38.6, -12), 0)
                .splineToConstantHeading(new Vector2d(45.5, -12), 0)
                .splineToConstantHeading(new Vector2d(42.5, -51), 3*pi/2)
                .strafeToLinearHeading(new Vector2d(59.6, -37.3), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder dropAndGrab = spline1.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -49.2), 3*pi/2)
                .waitSeconds(0.001);



        TrajectoryActionBuilder scoreFirstPickup = dropAndGrab.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(7, -41.1), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -49.2), 3*pi/2)
                .waitSeconds(0.001);


        TrajectoryActionBuilder scoreSecondPickup = pickupSideWallAfterScoreOne.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(4, -41.1), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -49.2), 3*pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder scoreThirdPickup = pickupSideWallAfterScoreTwo.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(1, -41.1), initialPose.heading)
                .waitSeconds(0.001);


        myBot.runAction(
                new SequentialAction(
                        dropSpecimen.build(),
                        spline1.build(),
                        dropAndGrab.build(),
                        scoreFirstPickup.build(),
                        pickupSideWallAfterScoreOne.build(),
                        scoreSecondPickup.build(),
                        pickupSideWallAfterScoreTwo.build(),
                        scoreThirdPickup.build()

                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }}