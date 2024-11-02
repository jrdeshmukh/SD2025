package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import  com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.google.blocks.ftcrobotcontroller.runtime.obsolete.TfodAccess;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.backtracking.Drive;
import org.firstinspires.ftc.teamcode.backtracking.MecDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class SDAutoTele extends OpMode {
    MecDrive drive;
    Slide slide;
    BBG gp1, gp2;

    Pose2d basket = new Pose2d(-54.1, -54.1, 5*Math.PI/4);
    FtcDashboard dash = FtcDashboard.getInstance();
    Claw claw;
    Wrist wrist;
    Action goAction;
    boolean autoDriving = false, lifting;

    double fw=0.0, strafe=0.00001, turn=0.0;
    private long lastGamepadUpdate = 0;


    double speedMod = 0.75;
    ElapsedTime start;
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        slide = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        drive = new MecDrive(hardwareMap, new Pose2d(-36.3912, -10.766, 0));
        goAction = claw.close();
        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
        start = new ElapsedTime();
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        autoDriving = false;

        if(gp2.dpad_up()) {
            runningActions.add(new SequentialAction(
                    claw.close(),
                    new InstantAction(() -> slide.runToPos(Slide.HIGH_BASKET-1030)),
                    new SleepAction(1.35),
                    wrist.wristDrop()
            ));
        }
        if(gp2.dpad_down()) {
            runningActions.add(new SequentialAction(
                    claw.open(),
                    new SleepAction(0.3),
                    wrist.wristHigh(),
                    new SleepAction(0.3),
                    new InstantAction(() -> slide.runToPos(Slide.BOTTOM-1030)),
                    new SleepAction(1),
                    wrist.wristPickup()
            ));
        }


        if(gp1.a()) {
            basket = drive.pose;
        }

        if (gp1.b()) {
            goAction = drive.actionBuilder(drive.pose).strafeToLinearHeading(basket.position, basket.heading).build();
            runningActions.add(goAction);
        }


        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                if(action.equals(goAction)) autoDriving = true;
                newActions.add(action);
            }
        }




        runningActions = newActions;
        dash.sendTelemetryPacket(packet);


        if(gamepad1.right_trigger>0.03) speedMod = 1;
        if(gamepad1.right_bumper) speedMod = 0.75;
        if(gamepad1.left_trigger>0.03) speedMod = 0.25;
        if(gamepad1.left_bumper) speedMod = 0.5;

        if (gamepad1.timestamp != lastGamepadUpdate) {
            lastGamepadUpdate = gamepad1.timestamp;
        }


        if ((Math.abs(gamepad1.left_stick_x) + Math.abs(gamepad1.left_stick_y) + Math.abs(gamepad1.right_stick_x))>0 || !autoDriving) {
            if (autoDriving) {
                runningActions.remove(goAction);
            }
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                    -gamepad1.left_stick_y*speedMod,
                    -gamepad1.left_stick_x*speedMod),
                    -gamepad1.right_stick_x*speedMod
            ));
        }

        drive.updatePoseEstimate();

        if(Math.abs(gamepad2.left_stick_y)>0) {
            slide.setPower(-gamepad2.left_stick_y);
            slide.runToPos(slide.slide.getCurrentPosition());
        }
        else {
            slide.setPow2();
        }


        if (gp2.right_bumper())                        claw.setPosition(0.81); //open
        if (gp2.left_bumper())                         claw.setPosition(0.39); //close

        if (gamepad2.left_trigger>0.01)                wrist.setPosition(Wrist.HIGH); //straight up
        if (gamepad2.right_trigger>0.01)               wrist.setPosition(Wrist.PICKUP); //pickup
        if (gp2.x())                                   wrist.setPosition(0.5); //sstraight out
        if (gp2.y())                                   wrist.setPosition(Wrist.DROP); //score, 60 degree above flat

        //telemetry.addData("slide current", slide.slide.getCurrentPosition());
        //telemetry.addData("speed mod: ", speedMod);
        //telemetry.addData("slide target", target);
        //telemetry.addData("claw pos", claw.claw.getPosition());
        //telemetry.addData("x: ", drive.pose.position.x);
        //telemetry.addData("y: ", drive.pose.position.y);
        //telemetry.addData("heading: ", Math.toDegrees(drive.pose.heading.toDouble()));
        //telemetry.addData("running actions: ", runningActions.size());
        //telemetry.addData("autoDriving: ", autoDriving);
        //telemetry.addData("time: ", new ElapsedTime().milliseconds()-start.milliseconds());
        //telemetry.update();
    }
}
