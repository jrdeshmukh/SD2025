package org.firstinspires.ftc.teamcode.autos;

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
        Pose2d initialPose = new Pose2d(10, -60, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        //first pixel x39.6719, y27.094
        //second pixel x49.892, y 27.18
        //pickup/drop x38.9178, y8.7159


        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;


        TrajectoryActionBuilder dropSpecimen = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(10, -37))
                .waitSeconds(0.1);

        TrajectoryActionBuilder pickup1 = dropSpecimen.fresh()
                .strafeTo(new Vector2d(10, -40))
                .strafeTo(new Vector2d(49.6719, -32.906))
                .waitSeconds(2);

        TrajectoryActionBuilder drop1 = pickup1.fresh()
                .strafeToLinearHeading(new Vector2d(48.9178, -48), 3*pi/2)
                .strafeTo(new Vector2d(48.9178, -51.2814))
                //.strafeTo(new Vector2d(-48, -41))
                //.strafeToLinearHeading(new Vector2d(-50.1, -50.1), 5*pi/4)
                //.strafeToLinearHeading(new Vector2d(-54.1, -54.1), 5*pi/4)
                .waitSeconds(0.1);

        TrajectoryActionBuilder pickup2 = drop1.fresh()
                .strafeTo(new Vector2d(48.9178, -48))
                .strafeToLinearHeading(new Vector2d(59.892, -32.82), pi/2)
                .waitSeconds(0.1);

        TrajectoryActionBuilder drop2  = pickup2.fresh()
                .strafeToLinearHeading(new Vector2d(48, -48), 3*pi/2)
                .strafeTo(new Vector2d(48, -51.2814))
                .waitSeconds(0.1);

        TrajectoryActionBuilder score1 = drop2.fresh()
                .strafeToLinearHeading(new Vector2d(8, -41), pi/2)
                .strafeTo(new Vector2d(8, -37))
                .waitSeconds(0.1);

        TrajectoryActionBuilder pickup3 = score1.fresh()
                .strafeToLinearHeading(new Vector2d(48.9178, -48), 3*pi/2)
                .strafeTo(new Vector2d(48.9178, -51.2814));


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(
                dropSpecimen.build(),
                pickup1.build(),
                drop1.build(),
                pickup2.build(),
                drop2.build(),
                score1.build(),
                pickup3.build()
                )
        );
    }
}
