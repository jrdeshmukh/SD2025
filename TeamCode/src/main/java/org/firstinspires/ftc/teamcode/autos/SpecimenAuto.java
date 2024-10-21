package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.Arrays;

@Autonomous()
public class SpecimenAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -64.5, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //first pixel x39.6719, y27.094
        //second pixel x49.892, y 27.18
        //pickup/drop x38.9178, y8.7159


        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;


        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -39))
                .waitSeconds(0.01);

        TrajectoryActionBuilder pickup1 = dropSpecimen.fresh()
                .strafeTo(new Vector2d(10, -45))
                .strafeTo(new Vector2d(49.2, -36.6))
                .waitSeconds(0.01);

        TrajectoryActionBuilder drop1 = pickup1.fresh()
                .strafeToLinearHeading(new Vector2d(48.9178, -53.7), 3*pi/2)
                .waitSeconds(0.01);

        TrajectoryActionBuilder pickup2 = drop1.fresh()
                .strafeTo(new Vector2d(48.9178, -48))
                .strafeToLinearHeading(new Vector2d(59.892, -37), pi/2)
                .waitSeconds(0.01);

        TrajectoryActionBuilder drop2  = pickup2.fresh()
                .strafeToLinearHeading(new Vector2d(48, -48), 3*pi/2)
                .strafeTo(new Vector2d(48, -55.7))
                .waitSeconds(0.01);

        TrajectoryActionBuilder score1 = drop2.fresh()
                .strafeToLinearHeading(new Vector2d(4, -45), pi/2)
                .strafeTo(new Vector2d(4, -38.2))
                .waitSeconds(0.01);



        TrajectoryActionBuilder drop3 = score1.fresh()
                .strafeToLinearHeading(new Vector2d(48.9178, -48), 3*pi/2)
                .strafeTo(new Vector2d(48.9178, -55.2))
                .waitSeconds(0.1);


        TrajectoryActionBuilder score2 = drop3.fresh()
                .strafeToLinearHeading(new Vector2d(-2, -45), pi/2)
                .strafeTo(new Vector2d(-2, -38.3))
                .waitSeconds(0.01);


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slide.setPow(),
                        new SequentialAction(
                                wrist.wristHigh(),
                                claw.close(),
                                slide.highRung(),
                                dropSpecimen.build(),
                                wrist.highRung(),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2),
                        new ParallelAction(
                                wrist.wristHigh(),
                                pickup1.build(),
                                slide.liftBottom(),
                                new SequentialAction(
                                        new SleepAction(0.5),
                                        wrist.wristPickup()
                                )
                        ),
                claw.close(),
                new SleepAction(0.5),
                new ParallelAction(
                        new InstantAction(() -> slide.runToPos(50)),
                        drop1.build()
                ),
                claw.open(),
                new SleepAction(0.2),
                slide.liftBottom(),
                pickup2.build(),
                claw.close(),
                new SleepAction(0.5),
                new InstantAction(() -> slide.runToPos(50)),
                drop2.build(),
                claw.open(),
                new SleepAction(0.5),
                wrist.wristSpecimen(),
                new SleepAction(0.7),
                claw.close(),
                new SleepAction(0.5),
                new ParallelAction(
                    score1.build(),
                    wrist.wristHigh()
                ),
                slide.highRung(),
                new SleepAction(0.7),
                wrist.highRung(),
                new SleepAction(0.35),
                claw.open(),
                new SleepAction(0.5),
                new InstantAction(() -> slide.runToPos(50)),
                new ParallelAction(
                        drop3.build(),
                        new SequentialAction(
                                new SleepAction(0.5),
                                wrist.wristSpecimen()
                        )
                ),
                claw.close(),
                new SleepAction(0.5),
                new InstantAction(() -> slide.runToPos(100)),
                wrist.wristHigh(),
                score2.build(),
                slide.highRung(),
                new SleepAction(0.7),
                wrist.highRung(),
                new SleepAction(0.35),
                claw.open()
                ))
        );
    }
}
