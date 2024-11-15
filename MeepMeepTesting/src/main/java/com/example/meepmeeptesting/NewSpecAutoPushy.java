package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public class NewSpecAutoPushy {
    public static void main(String[] args) {
        VelConstraint minConstraint = new MinVelConstraint(Arrays.asList(


                new TranslationalVelConstraint(50.0),
                new AngularVelConstraint(Math.PI / 2)

        ));
        MeepMeep meepMeep = new MeepMeep(800);
        double pi = Math.PI;

        Pose2d initialPose = new Pose2d(9, -60.5, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4554535190)
                .setStartPose(initialPose)
                .build();


        // vision here that outputs position
        int startPosition = 1;

        TrajectoryActionBuilder dropSpecimen = myBot.getDrive().actionBuilder(initialPose)
                .strafeTo(new Vector2d(9, -40.85))
                .waitSeconds(0.001);


        /*TrajectoryActionBuilder dropAll = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(27.5, -45))
                .splineToConstantHeading(new Vector2d(37.5, -7), 0)
                .splineToConstantHeading(new Vector2d(47.5, -7), 0)
                .strafeToConstantHeading(new Vector2d(47.5, -54))
                .splineToConstantHeading(new Vector2d(53, -10), 0)
                .strafeToConstantHeading(new Vector2d(53, -54))
                // .splineToConstantHeading(new Vector2d(50, -27), pi)
                //.splineToLinearHeading(new Pose2d(new Vector2d(60, -27), new Rotation2d(pi/2, 0)), 0)
                .strafeToLinearHeading(new Vector2d(58.5, -22.86), new Rotation2d(pi/2, 0))
                .waitSeconds(0.001);

        TrajectoryActionBuilder push1 = dropSpecimen.fresh()
                .strafeTo(new Vector2d(28.5, -41.7))
                .splineToConstantHeading(new Vector2d(46.5, -12), 0)
                //.strafeTo(new Vector2d(47.5, -12))
                //.setReversed(true)
                .splineToConstantHeading(new Vector2d(47.5, -54), 3*pi/2)
                .splineToConstantHeading(new Vector2d(59.6, -37.3), pi/2)
                .waitSeconds(0.001);


        TrajectoryActionBuilder dropBoth = dropSpecimen.fresh()
                .strafeTo(new Vector2d(37.5, -41.1))
                .strafeTo(new Vector2d(37.5, -7))
                .strafeTo(new Vector2d(47.5, -12))
                .strafeToConstantHeading(new Vector2d(47.5, -54))
                .strafeToLinearHeading(new Vector2d(50, -35), pi/2)
                .strafeToLinearHeading(new Vector2d(60, -53), 0)
                .waitSeconds(0.001);


        TrajectoryActionBuilder pickupSideWall = dropAll.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreFirstPickup = pickupSideWall.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(8, -40.85), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreSecondPickup = pickupSideWallAfterScoreOne.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(6, -40.85), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreThirdPickup = pickupSideWallAfterScoreTwo.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(14, -54.6), pi/2)
                .strafeTo(new Vector2d(1, -41.1))
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreThree = scoreThirdPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreFourthPickup = pickupSideWallAfterScoreThree.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(2, -42.85), initialPose.heading)
                .waitSeconds(0.001);*/





        TrajectoryActionBuilder dropAll = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(32, -45))
                .strafeToConstantHeading(new Vector2d(34, -12))
                .strafeToConstantHeading(new Vector2d(47.5, -12))
                .strafeToConstantHeading(new Vector2d(47.5, -51))
                .strafeToConstantHeading(new Vector2d(50, -12))
                .strafeToConstantHeading(new Vector2d(57.6, -12))
                .strafeToConstantHeading(new Vector2d(57.6, -54))
                // .splineToConstantHeading(new Vector2d(50, -27), pi)
                //.splineToLinearHeading(new Pose2d(new Vector2d(60, -27), new Rotation2d(pi/2, 0)), 0)
                .strafeToLinearHeading(new Vector2d(58.5, -22.86), new Rotation2d(pi/2, 0))
                .waitSeconds(0.001);



       /* TrajectoryActionBuilder pickupSideWall = dropAll.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);*/

        TrajectoryActionBuilder pickupSideWall = dropAll.fresh()
                .strafeToLinearHeading(new Vector2d(54.4, -55.17), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallNew = dropAll.fresh()
                .splineToSplineHeading(new Pose2d(new Vector2d(42, -49.8),-pi/2 ), 0).waitSeconds(0.001);



        TrajectoryActionBuilder scoreFirstPickup = pickupSideWallNew.fresh()
                .strafeToLinearHeading(new Vector2d(7, -41.1), initialPose.heading)
                .setReversed(true)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(54.4, -55.17), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallNewAfterScoreOne = scoreFirstPickup.fresh()
                .splineToSplineHeading(new Pose2d(new Vector2d(42, -49.8),-pi/2 ), 0).waitSeconds(0.001);


        myBot.runAction(
                new SequentialAction(
                        dropSpecimen.build(),
                        dropAll.build(),
                        pickupSideWallNew.build(),
                        scoreFirstPickup.build(),
                        pickupSideWallNewAfterScoreOne.build()
                        /*dropBoth.build(),
                        pickupSideWall.build(),
                        scoreFirstPickup.build(),
                        pickupSideWallAfterScoreOne.build(),
                        scoreSecondPickup.build(),
                        pickupSideWallAfterScoreTwo.build(),
                        scoreThirdPickup.build(),
                        pickupSideWallAfterScoreThree.build(),
                        scoreFourthPickup.build()*/

                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }}