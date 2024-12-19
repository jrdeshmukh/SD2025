package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.wrappers.PinpointDrive;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Spin;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

@Autonomous()
public class NewBasketAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-8.5, -63.7, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        Spin spin = new Spin(hardwareMap);
        double pi = Math.PI;


        TrajectoryActionBuilder toSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-8.5, -36.1))
                .waitSeconds(0.0001);

        TrajectoryActionBuilder toFirst = toSpecimen.fresh()
                .strafeTo(new Vector2d(-45.3, -42))
                .waitSeconds(0.001);

        TrajectoryActionBuilder scoreFirst = toFirst.fresh()
                .strafeToLinearHeading(new Vector2d(-46.6, -46.65), 5*pi/4)
                .waitSeconds(0.001);

        TrajectoryActionBuilder toSecond = scoreFirst.fresh()
                 .strafeToLinearHeading(new Vector2d(-54.2, -44.2), pi/2)
                 .waitSeconds(0.001);

        TrajectoryActionBuilder scoreSecond = toSecond.fresh()
                .strafeToLinearHeading(new Vector2d(-46.5, -46.65), 5*pi/4)
                .waitSeconds(0.001);




        wrist.setPosition(Wrist.HIGH);
        claw.setPosition(Claw.CLOSE);

        waitForStart();
        if(isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slide.setPow(),
                        new SequentialAction(
                                wrist.wristHigh(),
                                claw.close(),
                                slide.autoMove(1074),
                                toSpecimen.build(),
                                slide.autoMove(430),
                                new SleepAction(0.4),
                                claw.open(),
                                new SleepAction(0.2),
                                slide.liftBottom(),
                                new ParallelAction(
                                        toFirst.build(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                wrist.wristPickup()
                                        )
                                ),
                                claw.close(),
                                new SleepAction(0.5),
                                wrist.wristHigh(),
                                scoreFirst.build(),
                                slide.liftHigh(),
                                new SleepAction(1.5),
                                wrist.wristDrop(),
                                new SleepAction(0.3),
                                claw.open(),
                                new SleepAction(0.4),
                                wrist.wristHigh(),
                                new SleepAction(0.5),
                                slide.liftBottom(),
                                toSecond.build(),
                                wrist.wristPickup(),
                                new SleepAction(0.4),
                                claw.close(),
                                new SleepAction(0.4),
                                wrist.wristHigh(),
                                scoreSecond.build(),
                                slide.liftHigh(),
                                new SleepAction(1.5),
                                wrist.wristDrop(),
                                new SleepAction(0.3),
                                claw.open(),
                                new SleepAction(0.4),
                                wrist.wristHigh(),
                                new SleepAction(0.5),
                                slide.liftBottom()
                                )

                )
        );
    }
}
