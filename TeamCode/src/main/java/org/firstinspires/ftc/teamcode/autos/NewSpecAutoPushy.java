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
public class NewSpecAutoPushy extends LinearOpMode {
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


       /* TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -40.85))
                .waitSeconds(0.001);*/
        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -41.1))
                .waitSeconds(0.001);



        TrajectoryActionBuilder dropAll = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(34, -38))
                .strafeToConstantHeading(new Vector2d(34, -12))
                .strafeToConstantHeading(new Vector2d(47.5, -12))
                .strafeToConstantHeading(new Vector2d(47.5, -51))
                .strafeToConstantHeading(new Vector2d(47.5, -12))
                .strafeToConstantHeading(new Vector2d(57.6, -12))
                .strafeToConstantHeading(new Vector2d(57.6, -54))
                .strafeToConstantHeading(new Vector2d(57.6, -30))

                // .splineToConstantHeading(new Vector2d(50, -27), pi)
                //.splineToLinearHeading(new Pose2d(new Vector2d(60, -27), new Rotation2d(pi/2, 0)), 0)
                //.strafeToLinearHeading(new Vector2d(58.5, -22.86), new Rotation2d(pi/2, 0))
                .waitSeconds(0.001);

        TrajectoryActionBuilder push1 = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(28.5, -45))
                .splineToConstantHeading(new Vector2d(42.6, -12), 0)
                .splineToConstantHeading(new Vector2d(40.5, -12), 0)
                .splineToConstantHeading(new Vector2d(47.5, -51), 3*pi/2)
                .strafeToLinearHeading(new Vector2d(59.6, -37.3), pi/2)
                .waitSeconds(0.001);




       /* TrajectoryActionBuilder pickupSideWall = dropAll.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);*/


        TrajectoryActionBuilder pickupSideWallNew = push1.fresh()
                .strafeToLinearHeading(new Vector2d(45, -45), -pi/2)
                .strafeTo(new Vector2d(45, -49.8)).waitSeconds(0.001);



        TrajectoryActionBuilder scoreFirstPickup = pickupSideWallNew.fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(7, -41.1, pi/2+0.001), 0)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(54.4, -55.17), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallNewAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(45, -49.8), -pi/2).waitSeconds(0.001);


        TrajectoryActionBuilder scoreSecondPickup = pickupSideWallNewAfterScoreOne.fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(4, -41.1, pi/2+0.001), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(54.3, -55.17), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder pickupSideWallNewAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(45, -49.8), -pi/2).waitSeconds(0.001);


        TrajectoryActionBuilder scoreThirdPickup = pickupSideWallNewAfterScoreTwo.fresh()
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(1, -41.1, pi/2+0.001), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreThree = scoreThirdPickup.fresh()
                .strafeToLinearHeading(new Vector2d(54.2, -55.17), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder park = scoreThirdPickup.fresh()
                .strafeToConstantHeading(new Vector2d(59, -55)).waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallNewAfterScoreThree = scoreThirdPickup.fresh()
                .strafeToLinearHeading(new Vector2d(45, -49.8), -pi/2).waitSeconds(0.001);

        TrajectoryActionBuilder scoreFourthPickup = pickupSideWallNewAfterScoreThree.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-2, -41.1, pi/2+0.001), pi/2)
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
                                new InstantAction(() -> claw.setPosition(0.05)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2),
                                wrist.wristPickup(),
                                slide.liftBottom(),

                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                wrist.wristPickup(),
                                slide.liftBottom(),

                                dropAll.build(),
                                new InstantAction(() -> slide.runToPos(120)),
                                wrist.wristSpecimen(),
                                claw.open(),


                                pickupSideWallNew.build(),
                                //    new SleepAction(0.5),
                                new SleepAction(0.5),

                                claw.close(),
                                new SleepAction(0.15),
                                wrist.wristHigh(),

                                new ParallelAction(
                                        scoreFirstPickup.build(),
                                        wrist.wristSpecimen(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1520))
                                ),
                                new InstantAction(() -> wrist.setPosition(Wrist.SPECIMEN-0.08)),
                                new InstantAction(() -> claw.setPosition(0.07)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),

                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                new InstantAction(() -> slide.runToPos(120)),
                                pickupSideWallNewAfterScoreOne.build(),
                                claw.close(),
                                new SleepAction(0.15),


                                new ParallelAction(
                                        scoreSecondPickup.build(),
                                        wrist.wristSpecimen(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1520))
                                ),
                                new InstantAction(() -> wrist.setPosition(Wrist.SPECIMEN-0.08)),
                                new InstantAction(() -> claw.setPosition(0.07)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),



                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                new InstantAction(() -> slide.runToPos(120)),
                                pickupSideWallNewAfterScoreTwo.build(),
                                claw.close(),
                                new SleepAction(0.15),


                                new ParallelAction(
                                        scoreThirdPickup.build(),
                                        wrist.wristSpecimen(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1520))
                                ),
                                new InstantAction(() -> wrist.setPosition(Wrist.SPECIMEN-0.08)),
                                new InstantAction(() -> claw.setPosition(0.07)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),





                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                park.build()
                               /* new InstantAction(() -> slide.runToPos(120)),
                                claw.close(),
                                new SleepAction(0.5)*/


                              /*  new ParallelAction(
                                        scoreFourthPickup.build(),
                                        wrist.wristSpecimen(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1520))
                                ),
                                new InstantAction(() -> wrist.setPosition(Wrist.SPECIMEN-0.08)),
                                new InstantAction(() -> claw.setPosition(0.07)),
                                new SleepAction(0.2),
                                new InstantAction(() -> slide.runToPos(870)),
                                new SleepAction(0.5),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom()*/
                        )
                )
        );
    }
}
