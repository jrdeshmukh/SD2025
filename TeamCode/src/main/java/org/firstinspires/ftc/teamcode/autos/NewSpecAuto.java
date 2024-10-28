package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
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
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;
//hi
@Autonomous()
public class NewSpecAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(10, -64.5, Math.toRadians(90));
        MecDrive drive = new MecDrive(hardwareMap, initialPose);

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

        TrajectoryActionBuilder pickup1 = dropSpecimen.fresh()
                //.strafeTo(new Vector2d(10, -40))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(39, -26.44, 0), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder drop1 = pickup1.fresh()
                .strafeTo(new Vector2d(39, -59))
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickup2 = drop1.fresh()
                .strafeTo(new Vector2d(39, -40))
                .strafeTo(new Vector2d(48.87, -26.082))
                .waitSeconds(0.001);

        TrajectoryActionBuilder pickup3  = pickup2.fresh()
                .strafeToLinearHeading(new Vector2d(48, -55.7), 3*pi/2)
                .waitSeconds(0.001);


        TrajectoryActionBuilder score1 = pickup3.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(8, -34, pi/2), pi/2)
                .waitSeconds(0.001);

        TrajectoryActionBuilder picup2 = score1.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, -59, 3*pi/2), 3*pi/2).waitSeconds(0.001);


        TrajectoryActionBuilder score3 = picup2.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(5, -34, pi/2), pi/2).waitSeconds(0.001);

        TrajectoryActionBuilder pickup5 = score3.fresh()
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(48, -59, 3*pi/2), 3*pi/2).waitSeconds(0.001);


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
                        pickup1.build(),
                        new SleepAction(0.2),
                        claw.close(),
                        new SleepAction(0.5),
                        drop1.build(),
                        new InstantAction(() -> slide.runToPos(50)),
                        claw.open(),
                        pickup2.build(),
                        claw.close(),
                        new SleepAction(0.5),
                        pickup3.build()

                )
            )
        );
    }
}
