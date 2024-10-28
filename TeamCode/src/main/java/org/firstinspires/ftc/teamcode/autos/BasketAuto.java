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
import org.firstinspires.ftc.teamcode.backtracking.Drive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.Arrays;

@Autonomous()
public class BasketAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-10, -64.5, Math.toRadians(90));
        Drive drive = new Drive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0)
        ));

        TrajectoryActionBuilder toSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(-10, -39))
                .waitSeconds(0.01);



        TrajectoryActionBuilder toFirst = toSpecimen.fresh()
                .strafeTo(new Vector2d(-10, -45))
                .strafeTo(new Vector2d(-49, -36.5))
                .waitSeconds(0.01);

        TrajectoryActionBuilder toBasket1 = toFirst.fresh()
                .strafeTo(new Vector2d(-48, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-54.6, -54.6), 5*pi/4)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toSecond = toBasket1.fresh()
                //.strafeTo(new Vector2d(-50.1, -50.1))
                .strafeToLinearHeading(new Vector2d(-59, -36.5), pi/2)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toBasket2 = toSecond.fresh()
                .strafeTo(new Vector2d(-58.5, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toThird = toBasket2.fresh()
                .strafeTo(new Vector2d(-50.1, -50.1))
                .strafeToLinearHeading(new Vector2d(-59.5, -26.5), pi)
                .waitSeconds(0.01);

        TrajectoryActionBuilder backup = toThird.fresh()
                .strafeTo(new Vector2d(-55, -26.9))
               // .strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(0.01);



        // x -26.3912 y 53.7334
        TrajectoryActionBuilder park = backup.fresh()
                        .strafeToLinearHeading(new Vector2d(-36.3912, -10.766), 0)
                .waitSeconds(0.1);

        TrajectoryActionBuilder park2 = park.fresh()
                        .strafeTo(new Vector2d(-21, -10.766));

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                    new InstantAction(() -> drive.updatePoseEstimate()),
                    slide.setPow(),
                    new SequentialAction(
                        wrist.wristHigh(),
                        claw.close(),
                        slide.highRung(),
                        toSpecimen.build(),
                        wrist.highRung(),
                        new SleepAction(0.35),
                        claw.open(),
                        new SleepAction(0.2),
                        new ParallelAction(
                            wrist.wristHigh(),
                            toFirst.build(),
                            slide.liftBottom(),
                            new SequentialAction(
                                 new SleepAction(0.5),
                                 wrist.wristPickup()
                            )
                        ),
                        claw.close(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                wrist.wristHigh(),
                                toBasket1.build()
                        ),
                        slide.liftHigh(),
                        new SleepAction(1.2),
                        wrist.wristDrop(),
                        new SleepAction(0.2),
                        claw.open(),
                        new SleepAction(0.3),
                        wrist.wristHigh(),
                        new SleepAction(0.3),
                        new ParallelAction(
                                slide.liftBottom(),
                                toSecond.build(),
                                wrist.wristPickup()
                        ),
                        claw.close(),
                        new SleepAction(0.5),
                        new ParallelAction(
                                wrist.wristHigh(),
                                toBasket2.build()
                        ),
                        slide.liftHigh(),
                        new SleepAction(1.2),
                        wrist.wristDrop(),
                        new SleepAction(0.2),
                        claw.open(),
                        new SleepAction(0.3),
                        wrist.wristHigh(),
                        new SleepAction(0.3),
                        slide.liftBottom(),
                        new ParallelAction(
                                wrist.wristPickup(),
                                toThird.build()
                        ),
                        claw.close(),
                        new SleepAction(0.4),
                        new InstantAction( () -> slide.runToPos(60)),
                        new SleepAction(0.1),
                        new ParallelAction(
                                backup.build(),
                                new SequentialAction(
                                        new SleepAction(0.7),
                                        wrist.wristHigh()
                                )
                        ),
                        slide.liftHigh(),
                        new SleepAction(1.2),
                        wrist.wristDrop(),
                        new SleepAction(0.2),
                        claw.open(),
                        new SleepAction(0.3),
                        wrist.wristHigh(),
                        new SleepAction(0.3),
                        slide.liftBottom(),
                        new ParallelAction(
                              park.build(),
                              wrist.wristHigh()
                        ),
                        new InstantAction(() -> slide.runToPos(1270)),
                        new SleepAction(0.3),
                         park2.build(),
                         new InstantAction(() -> slide.runToPos(1030))
                    )
                )

        );
    }
}
