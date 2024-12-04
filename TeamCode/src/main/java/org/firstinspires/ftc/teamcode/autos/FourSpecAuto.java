package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.backtracking.Drive;
import org.firstinspires.ftc.teamcode.backtracking.MecDrive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.PinpointDrive;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;
//hi
@Autonomous()
public class FourSpecAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -64.5, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);

        //first pixel x39.6719, y27.094
        //second pixel x49.892, y 27.18
        //pickup/drop x38.9178, y8.7159


        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;
        double barY = -41.7;
        int highHeight = 1520;
        int lowHeight = 830;
        int pickupHeight = 150;


       /* TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -40.85))
                .waitSeconds(0.001);*/
        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, barY))
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
                .splineToLinearHeading(new Pose2d(47.5, -12, pi/2-0.01), 0)
                .strafeTo(new Vector2d(42.5, -51))
                .strafeToLinearHeading(new Vector2d(59.6, -37.3), pi/2)
                .waitSeconds(0.001);



        TrajectoryActionBuilder dropAndGrab = spline1.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -47.2), 3*pi/2)
                .waitSeconds(0.001);



        TrajectoryActionBuilder scoreFirstPickup = dropAndGrab.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(7, -41.1), pi/2+0.001)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -49.8), 3*pi/2)
                .waitSeconds(0.001);


        TrajectoryActionBuilder scoreSecondPickup = pickupSideWallAfterScoreOne.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(4, -41.1), pi/2+0.001)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(52.93, -49.7), 3*pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder scoreThirdPickup = pickupSideWallAfterScoreTwo.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(1, -41.1), pi/2+0.001)
                .waitSeconds(0.001);

        Actions.runBlocking(
                new ParallelAction(
                        wrist.wristHigh(),
                        claw.close()
                )
        );


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slide.setPow(),
                        new SequentialAction(
                                wrist.wristSpecimen(),
                                claw.close(),
                                new ParallelAction(
                                        dropSpecimen.build(),
                                        wrist.wristSpecimen(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1520))
                                ),
                                new InstantAction(() -> wrist.setPosition(Wrist.SPECIMEN-0.08)),
                                new InstantAction(() -> claw.setPosition(Claw.CLOSE-0.03)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2),
                                slide.liftBottom(),
                                new ParallelAction(
                                    spline1.build(),
                                    new SequentialAction(
                                        new SleepAction(1),
                                        wrist.wristPickup()
                                    )
                                ),
                                new SleepAction(0.2),
                                claw.close(),
                                new SleepAction(0.3),
                                new InstantAction(() -> slide.runToPos(100)),
                                dropAndGrab.build(),
                                claw.open(),
                                new SleepAction(0.25),
                                new InstantAction(() -> slide.runToPos(pickupHeight+20)),
                                new ParallelAction(
                                    wrist.wristSpecimen(),
                                    new SequentialAction(
                                        new SleepAction(0.5),
                                        claw.close(),
                                        new SleepAction(0.5),

                                        new InstantAction(() -> slide.runToPos(highHeight)),
                                        new SleepAction(0.6),
                                        scoreFirstPickup.build(),
                                        new InstantAction(() -> slide.runToPos(lowHeight)),
                                        new SleepAction(0.35),
                                        claw.open(),
                                        new SleepAction(0.2), //try to remove

                                        new InstantAction(() -> slide.runToPos(pickupHeight+50)),
                                        pickupSideWallAfterScoreOne.build(),
                                        new SleepAction(0.5),
                                        claw.close(),
                                        new SleepAction(0.5),
                                        new InstantAction(() -> slide.runToPos(highHeight+50)),
                                        new SleepAction(0.6),
                                        scoreSecondPickup.build(),
                                        new InstantAction(() -> slide.runToPos(lowHeight)),
                                        new SleepAction(0.35),
                                        claw.open(),
                                        new SleepAction(0.2), //try to remove
                                        new InstantAction(() -> slide.runToPos(pickupHeight+50)),
                                        pickupSideWallAfterScoreTwo.build(),
                                        new SleepAction(0.3),
                                        claw.close(),
                                        new SleepAction(0.6),
                                        new InstantAction(() -> slide.runToPos(highHeight+170)),
                                        new SleepAction(0.6),
                                        scoreThirdPickup.build(),
                                        new InstantAction(() -> slide.runToPos(lowHeight)),
                                        new SleepAction(0.35),
                                        claw.open(),
                                        new SleepAction(0.2)
                                    )
                                ) //try to remove





                                /*wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                pickupSideWallAfterScoreThree.build(),
                                new InstantAction(() -> slide.runToPos(120)),
                                claw.close(),
                                new SleepAction(0.5),


                                new ParallelAction(
                                        scoreFourthPickup.build(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        wrist.wristSpecimen()
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom()*/
                        )
                )
        );
    }
}
