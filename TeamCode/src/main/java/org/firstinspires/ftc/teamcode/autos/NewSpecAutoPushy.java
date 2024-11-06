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


        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(10, -40.85))
                .waitSeconds(0.001);


        TrajectoryActionBuilder dropAll = dropSpecimen.fresh()
                .strafeToConstantHeading(new Vector2d(27.5, -45))
                .splineToConstantHeading(new Vector2d(47.5, -10), 0)
                .strafeToConstantHeading(new Vector2d(47.5, -60))
                .splineToConstantHeading(new Vector2d(57.5, -10), 0)
                .strafeToConstantHeading(new Vector2d(57.5, -60))
                // .splineToConstantHeading(new Vector2d(50, -27), pi)
                //.splineToLinearHeading(new Pose2d(new Vector2d(60, -27), new Rotation2d(pi/2, 0)), 0)
                .strafeToLinearHeading(new Vector2d(60, -27), new Rotation2d(pi/2, 0))
                .waitSeconds(0.001);


        TrajectoryActionBuilder pickupSideWall = dropAll.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreFirstPickup = pickupSideWall.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(8, -41.35), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreOne = scoreFirstPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreSecondPickup = pickupSideWallAfterScoreOne.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(6, -41.85), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreTwo = scoreSecondPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreThirdPickup = pickupSideWallAfterScoreTwo.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(4, -42.35), initialPose.heading)
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickupSideWallAfterScoreThree = scoreThirdPickup.fresh()
                .strafeToLinearHeading(new Vector2d(initialPose.position.x + 46, initialPose.position.y + 4.6), new Rotation2d(3*pi/2, 0)).waitSeconds(0.001);


        TrajectoryActionBuilder scoreFourthPickup = pickupSideWallAfterScoreThree.fresh()
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(2, -42.85), initialPose.heading)
                .waitSeconds(0.001);


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        slide.setPow(),
                        new SequentialAction(
                                new ParallelAction(
                                        dropSpecimen.build(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        new InstantAction(() -> wrist.setPosition(0.5))
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),

                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                wrist.wristPickup(),
                                slide.liftBottom(),
                                dropAll.build(),
                                claw.close(),

                                new SleepAction(0.5),
                                new InstantAction(() -> slide.runToPos(75)),


                                pickupSideWall.build(),
                                claw.open(),
                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                //    new SleepAction(0.5),

                                claw.close(),
                                new SleepAction(0.5),
                                new InstantAction(() -> slide.runToPos(100)),
                                wrist.wristHigh(),

                                new ParallelAction(
                                        scoreFirstPickup.build(),
                                        claw.close(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        new InstantAction(() -> wrist.setPosition(0.5))
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),

                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                pickupSideWallAfterScoreOne.build(),
                                claw.close(),
                                new SleepAction(0.5),


                                new ParallelAction(
                                        scoreSecondPickup.build(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        new InstantAction(() -> wrist.setPosition(0.5))
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),



                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                pickupSideWallAfterScoreTwo.build(),
                                claw.close(),
                                new SleepAction(0.5),


                                new ParallelAction(
                                        scoreThirdPickup.build(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        new InstantAction(() -> wrist.setPosition(0.5))
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom(),





                                wrist.wristSpecimen(),
                                new InstantAction(() -> slide.runToPos(50)),
                                claw.open(),
                                pickupSideWallAfterScoreThree.build(),
                                claw.close(),
                                new SleepAction(0.5),


                                new ParallelAction(
                                        scoreFourthPickup.build(),
                                        new InstantAction(() -> slide.runToPos(1570)),
                                        new InstantAction(() -> wrist.setPosition(0.5))
                                ),
                                new InstantAction(() -> slide.runToPos(910)),
                                new SleepAction(0.35),
                                claw.open(),
                                new SleepAction(0.2), //try to remove
                                wrist.wristPickup(),
                                slide.liftBottom()
                        )
                )
        );
    }
}
