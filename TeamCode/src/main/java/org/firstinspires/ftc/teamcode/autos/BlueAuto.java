package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

@Autonomous()
public class BlueAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        Claw claw = new Claw(hardwareMap);
        Wrist wrist = new Wrist(hardwareMap);
        Slide slide = new Slide(hardwareMap);
        double pi = Math.PI;

        Action toSpecimen = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(25.7, 0))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(new ParallelAction(slide.setPow(), new SequentialAction(
                wrist.wristHigh(),
                claw.close(),
                slide.highRung(),
                toSpecimen,
                wrist.highRung(),
                new SleepAction(2.0),
                claw.open()
                //wrist.wristHigh(),
                //slide.liftBottom()
                ))

        );
    }
}
