package org.firstinspires.ftc.teamcode.pedroPathing.autos;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

@Autonomous()
public class SpecPedrauto extends LinearOpMode {

    Follower follower;




    @Override
    public void runOpMode() {
        Pose startPose = new Pose(8.1, 81.3, 0);
        follower = new Follower(hardwareMap, startPose);
        Slide slide = new Slide(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);

        Path drop = new Path(
                // Line 1
                new BezierLine(
                        new Point(8.100, 81.300, Point.CARTESIAN),
                        new Point(33.300, 81.300, Point.CARTESIAN)
                )
        );
        drop.setConstantHeadingInterpolation(0);

        Path goFirst = new Path(
                // Line 2
                new BezierLine(
                        new Point(33.300, 81.300, Point.CARTESIAN),
                        new Point(33.900, 118.00, Point.CARTESIAN)
                )
        ).setConstantHeadingInterpolation(0);

        Path scoreFirst = new Path(
                // Line 3
                new BezierLine(
                        new Point(33.300, 118.000, Point.CARTESIAN),
                        new Point(20.000, 124.000, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        Path goSecond = new Path(
                // Line 4
                new BezierLine(
                        new Point(20.700, 124.900, Point.CARTESIAN),
                        new Point(32.400, 129.708, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0));


        Path scoreSecond = new Path(
                // Line 5
                new BezierLine(
                        new Point(32.400, 129.708, Point.CARTESIAN),
                        new Point(18.000, 124.000, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45));

        Path toThird = new Path(
                // Line 6
                new BezierLine(
                        new Point(18.000, 124.000, Point.CARTESIAN),
                        new Point(42.100, 128.900, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(90));

        Path scoreThird = new Path(
                // Line 7
                new BezierCurve(
                        new Point(42.100, 128.900, Point.CARTESIAN),
                        new Point(41.782, 114.342, Point.CARTESIAN),
                        new Point(19.000, 123.000, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(-45));

        Path park = new Path(
                // Line 8
                new BezierLine(
                        new Point(19.000, 123.000, Point.CARTESIAN),
                        new Point(60.700, 108.300, Point.CARTESIAN)
                )
        )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(270));



        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new ParallelAction(
                        follower.followerUpdate(),
                        slide.setPow(),
                        new SequentialAction(
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_UP)),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                                mechanism.worm.autoMove(1290),
                                follower.follow(drop),
                                mechanism.slide.autoMove(1220),
                                new SleepAction(1), //1
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.2),
                                mechanism.slide.liftBottom(),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        follower.follow(goFirst),
                                        new SequentialAction(
                                                new SleepAction(0.4),
                                                mechanism.worm.autoMove(695),
                                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_DOWN))
                                        )
                                ),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                                new SleepAction(0.5),
                                mechanism.worm.autoMove(2827),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_UP)),
                                follower.follow(scoreFirst),
                                mechanism.slide.autoMove(2450),
                                new SleepAction(0.8),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_DOWN)),
                                new SleepAction(0.3),
                                mechanism.slide.liftBottom(),
                                new SleepAction(0.7),
                                mechanism.worm.autoMove(695),
                                follower.follow(goSecond),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                                new SleepAction(0.5),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_UP)),
                                mechanism.worm.autoMove(2827),
                                follower.follow(scoreSecond),
                                mechanism.slide.autoMove(2450),
                                new SleepAction(0.8),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_DOWN)),
                                new SleepAction(0.3),
                                mechanism.slide.liftBottom(),
                                new SleepAction(0.7),
                                mechanism.worm.autoMove(740),
                                //new InstantAction(() -> mechanism.setSpin(Mechanism.SPIN_90)),
                                follower.follow(toThird),
                                new InstantAction(() -> mechanism.setSpin(Mechanism.SPIN_90)),
                                new SleepAction(0.1),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_CLOSE)),
                                new SleepAction(0.5),
                                mechanism.worm.autoMove(2827),
                                new InstantAction(() -> mechanism.setSpin(Mechanism.SPIN_0)),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_UP)),
                                follower.follow(scoreThird),
                                mechanism.slide.autoMove(2450),
                                new SleepAction(0.8),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_SCORE)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setClaw(Mechanism.CLAW_OPEN)),
                                new SleepAction(0.3),
                                new InstantAction(() -> mechanism.setChosenAngle(Mechanism.WRIST_DOWN)),
                                new SleepAction(0.3),
                                mechanism.slide.liftBottom(),
                                new SleepAction(0.7),
                                mechanism.worm.autoMove(800),
                                follower.follow(park),
                                mechanism.slide.autoMove(1390),
                                mechanism.slide.waitUntilDone(),
                                mechanism.worm.autoMove(1120)

                        )
                )
        );

    }

}
