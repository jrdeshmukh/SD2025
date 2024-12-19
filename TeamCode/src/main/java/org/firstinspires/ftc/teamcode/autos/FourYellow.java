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
import org.firstinspires.ftc.teamcode.backtracking.MecDrive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.PinpointDrive;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.Arrays;

@Autonomous()
public class FourYellow extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-27.95, -62.88, Math.toRadians(180));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);

        MecanumDrive.PARAMS.maxWheelVel = 50;

        double pi = Math.PI;


       TrajectoryActionBuilder scorePreload = drive.actionBuilder(drive.pose)
               //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
               .strafeToLinearHeading(new Vector2d(-48.2, -49.2), 5*pi/4)
               .waitSeconds(0.001);




        TrajectoryActionBuilder toFirst = scorePreload.fresh()
                .strafeToLinearHeading(new Vector2d(-49.2, -38.5), pi/2)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toBasket1 = toFirst.fresh()
                .strafeTo(new Vector2d(-48, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-48, -50), 5*pi/4)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toSecond = toBasket1.fresh()
                //.strafeTo(new Vector2d(-50.1, -50.1))
                .strafeToLinearHeading(new Vector2d(-58.1, -36.8), pi/2)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toBasket2 = toSecond.fresh()
                .strafeTo(new Vector2d(-58.5, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-47.8, -49.8), 5*pi/4)
                .waitSeconds(0.01);

        TrajectoryActionBuilder toThird = toBasket2.fresh()
                .strafeToLinearHeading(new Vector2d(-50.4, -25), pi)
                .strafeToLinearHeading(new Vector2d(-54.4, -25), pi)
                .waitSeconds(0.01);

        TrajectoryActionBuilder backup = toThird.fresh()
                .strafeTo(new Vector2d(-55, -26.9))
                // .strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                .strafeToLinearHeading(new Vector2d(-48, -50), 5*pi/4)
                .waitSeconds(0.01);


        // x -26.3912 y 53.7334
        TrajectoryActionBuilder park = backup.fresh()
                .strafeToLinearHeading(new Vector2d(-36.3912, -5), 0)
                .waitSeconds(0.1);

        TrajectoryActionBuilder park2 = park.fresh()
                .strafeTo(new Vector2d(-21, -5));

        Actions.runBlocking(
                new SequentialAction(
                        claw.close(),
                        wrist.wristHigh()
                )
        );
        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slide.setPow(),
                        new SequentialAction(
                                wrist.wristHigh(),
                                claw.close(),
                                scorePreload.build(),
                                slide.liftHigh(),
                                new SleepAction(1.7),
                                wrist.wristDrop(),
                                new SleepAction(0.2),
                                claw.open(),
                                new SleepAction(0.3),
                                wrist.wristHigh(),
                                new ParallelAction(
                                        toFirst.build(),
                                        new SequentialAction(
                                                new SleepAction(0.2),
                                                new InstantAction( () -> slide.runToPos(-30)),
                                                new SleepAction(0.05),
                                                wrist.wristPickup()
                                        )
                                ),
                                new SleepAction(0.7),
                                claw.close(),
                                new SleepAction(0.5),
                                new ParallelAction(
                                        wrist.wristHigh(),
                                        toBasket1.build()
                                ),
                                slide.liftHigh(),
                                new SleepAction(1.5),
                                wrist.wristDrop(),
                                new SleepAction(0.4),
                                claw.open(),
                                new SleepAction(0.3),
                                wrist.wristHigh(),
                                new ParallelAction(
                                        new SequentialAction(
                                                new SleepAction(0.3),
                                                new InstantAction( () -> slide.runToPos(-30)),
                                                wrist.wristPickup()
                                        ),
                                        toSecond.build()
                                ),
                                new SleepAction(0.2),
                                claw.close(),
                                new SleepAction(0.8),
                                new ParallelAction(
                                        wrist.wristHigh(),
                                        toBasket2.build()
                                ),
                                new InstantAction( () -> slide.runToPos(3200)),
                                new SleepAction(1.7),
                                wrist.wristDrop(),
                                new SleepAction(0.4),
                                claw.open(),
                                new SleepAction(0.3),
                                wrist.wristHigh(),
                                new ParallelAction(
                                       toThird.build(),
                                        new SequentialAction(
                                                new InstantAction( () -> slide.runToPos(-40)),
                                                new SleepAction(0.3),
                                                wrist.wristPickup()
                                        )
                                ),
                                new SleepAction(0.1),
                                claw.close(),
                                new SleepAction(0.8),
                                new InstantAction( () -> slide.runToPos(60)),
                                new SleepAction(0.1),
                                new ParallelAction(
                                        backup.build(),
                                        new SequentialAction(
                                                new SleepAction(0.7),
                                                wrist.wristHigh()
                                        )
                                ),
                                new InstantAction( () -> slide.runToPos(3250)),
                                new SleepAction(1.8),
                                wrist.wristDrop(),
                                new SleepAction(0.4),
                                claw.open(),
                                new SleepAction(0.3),
                                wrist.wristHigh(),
                                new SleepAction(0.5),
                                new InstantAction(() -> slide.runToPos(800)),
                                wrist.wristHigh(),
                                new ParallelAction(
                                        park.build(),
                                        new SequentialAction(
                                                new SleepAction(0.5),
                                                wrist.wristPickup()
                                        )
                                ),
                                park2.build()
                        )
                )

        );
    }
}
