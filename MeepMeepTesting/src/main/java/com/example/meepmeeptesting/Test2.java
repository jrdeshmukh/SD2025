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

public class Test2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);
        double pi = Math.PI;

        Pose2d initialPose = new Pose2d(-10, -64.5, Math.toRadians(90));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 12.4554535190)
                .setStartPose(initialPose)
                .build();


        // vision here that outputs position
        int startPosition = 1;

        TrajectoryActionBuilder dropSpecimen = myBot.getDrive().actionBuilder(initialPose)
                .waitSeconds(2)
                .strafeTo(new Vector2d(-10, -34))
                .waitSeconds(2);

        TrajectoryActionBuilder go1 = dropSpecimen.fresh()
                //.splineToConstantHeading(new Vector2d(-48, -37), 3*pi/2)
                //.strafeTo(new Vector2d(-10, -45))
                .strafeTo(new Vector2d(-48, -37))
                .waitSeconds(2);

        TrajectoryActionBuilder basket1 = go1.fresh()
                .strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                //.strafeTo(new Vector2d(-48, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                //.strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(2);

        TrajectoryActionBuilder go2 = basket1.fresh()
                //.strafeTo(new Vector2d(-50.1, -50.1))
                .strafeToLinearHeading(new Vector2d(-58.5, -38), pi/2)

                //.strafeToLinearHeading(new Vector2d(-58.5, -38), pi/2)
                .waitSeconds(2);

        TrajectoryActionBuilder basket2 = go2.fresh()
                .strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)

                //.strafeTo(new Vector2d(-58.5, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                //.strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(2);

        TrajectoryActionBuilder go3 = basket2.fresh()
                //.strafeTo(new Vector2d(-50.1, -50.1))
                .strafeToLinearHeading(new Vector2d(-57.6, -24), pi)
                .waitSeconds(2);

        TrajectoryActionBuilder basket3 = go3.fresh()
                //.strafeTo(new Vector2d(-50, -24))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(2);

        TrajectoryActionBuilder park = basket3.fresh()
                .strafeToLinearHeading(new Vector2d(-40, -12), 0)
                .waitSeconds(2);





        Action trajectoryActionChosen;
        trajectoryActionChosen = dropSpecimen.build();


        myBot.runAction(
                new SequentialAction(
                        dropSpecimen.build(),
                        go1.build(),
                        basket1.build(),
                        go2.build(),
                        basket2.build(),
                        go3.build(),
                        basket3.build(),
                        park.build()
                )
        );


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }}