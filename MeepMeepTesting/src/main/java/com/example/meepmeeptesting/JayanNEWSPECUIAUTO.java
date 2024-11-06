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

        TrajectoryActionBuilder pickup1 = dropSpecimen.fresh()
                //.strafeTo(new Vector2d(10, -40))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(32, -35, 0), pi/2)
                .splineToConstantHeading(new Vector2d(45, -10), 0)
                .waitSeconds(0.001);

        TrajectoryActionBuilder drop1 = pickup1.fresh()
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickup2 = drop1.fresh()
                .waitSeconds(0.001);

        TrajectoryActionBuilder drop2 = pickup2.fresh()
                .strafeTo(new Vector2d(38, -59))
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupWallSidePixel  = drop2.fresh()
                .strafeToLinearHeading(new Vector2d(58, -26), new Rotation2d(3*pi/2, 0))
                .waitSeconds(0.001);

        TrajectoryActionBuilder dropSideWallPixel = pickupWallSidePixel.fresh()
                .strafeTo(new Vector2d(38, -59))
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWall = drop2.fresh()
                .waitSeconds(0.001);


        TrajectoryActionBuilder score1 = drop2.fresh()
                .setReversed(true)
                .waitSeconds(0.001);

        TrajectoryActionBuilder picup2 = score1.fresh()
                .setReversed(true)
                .waitSeconds(0.001);


        TrajectoryActionBuilder score3 = picup2.fresh()
                .setReversed(true)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickup5 = score3.fresh()
                .setReversed(true)
                .waitSeconds(0.001);

        myBot.runAction(
                new SequentialAction(
                        dropSpecimen.build(),
                        pickup1.build(),
                        drop1.build(),
                        pickup2.build(),
                        drop2.build(),
                        pickupWallSidePixel.build(),
                        dropSideWallPixel.build(),
                        pickupSideWall.build()

                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }}