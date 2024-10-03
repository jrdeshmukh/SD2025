package org.firstinspires.ftc.teamcode.autos;

import androidx.annotation.NonNull;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
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
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d startPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;

        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0)
        ));

        TrajectoryActionBuilder toSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(25.7, 0));



        TrajectoryActionBuilder toFirst = drive.actionBuilder(new Pose2d(25.7, 0, 0))
                .lineToX(20);

        TrajectoryActionBuilder toFirst2 = drive.actionBuilder(new Pose2d(20, 0, 0))
                .strafeTo(new Vector2d(20,20));

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(
                slide.setPow(),
                new SequentialAction(
                wrist.wristHigh(),
                claw.close(),
                slide.highRung(),
                toSpecimen.build(),
                wrist.highRung(),
                new SleepAction(1.0),
                claw.open(),
                new SleepAction(1.0),
                wrist.wristHigh(),
                new SleepAction(1.0),
                slide.liftBottom(),
                toFirst.build(),
                toFirst2.build()
                ))

        );
    }
}
