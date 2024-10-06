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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.ArrayList;
import java.util.List;

@TeleOp()
public class SDAutoTele extends OpMode {
    MecanumDrive drive;
    Slide slide;
    BBG gp1, gp2;
    List<LynxModule> allHubs;
    Action pickupAction, dropAction;
    Pose2d basket = new Pose2d(0,0,0);
    FtcDashboard dash = FtcDashboard.getInstance();
    Claw claw;
    Wrist wrist;
    Action goAction;
    boolean autoDriving;


    int target = 20;
    double curpow = 0;
    double speedMod = 0.75;
    List<Action> runningActions = new ArrayList<>();

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        slide = new Slide(hardwareMap);
        basket = new Pose2d(0,0,0);
        claw = new Claw(hardwareMap);
        wrist = new Wrist(hardwareMap);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        goAction = claw.close();
        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();
        autoDriving = false;

        if(gamepad2.dpad_left) {
            runningActions.add(new SequentialAction(
                    wrist.wristHigh(),
                    new InstantAction(() -> slide.runToPos(Slide.HIGH_BASKET)),
                    new SleepAction(1.5),
                    wrist.wristDrop()
            ));
        }
        if(gamepad2.dpad_right) {
            runningActions.add(new SequentialAction(
                    wrist.wristHigh(),
                    new SleepAction(0.5),
                    new InstantAction(() -> slide.runToPos(Slide.BOTTOM)),
                    wrist.wristPickup()));
        }


        if(gamepad1.a) {
            basket = drive.pose;
        }

        if (gamepad1.b) {
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

        if (Math.abs(gamepad1.left_stick_x + gamepad2.left_stick_y + gamepad2.right_stick_x)>0 || !autoDriving) {
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
            telemetry.addData("pp", 1);
            slide.setPower(gamepad2.left_stick_y);
            slide.runToPos(slide.slide.getCurrentPosition());
        }
        else {
            slide.setPow2();
        }


        if (gp2.right_bumper())                        claw.setPosition(0.81); //open
        if (gp2.left_bumper())                         claw.setPosition(0.39); //close

        if (gamepad2.left_trigger>0.01)                wrist.setPosition(0.86); //straight up
        if (gamepad2.right_trigger>0.01)               wrist.setPosition(0.1494); //pickup
        if (gp2.x())                                   wrist.setPosition(0.5089); //sstraight out
        if (gp2.y())                                   wrist.setPosition(0.7489); //score, 60 degree above flat

        if (gamepad2.dpad_down)                        slide.runToPos(Slide.BOTTOM);
        if (gamepad2.dpad_up)                          {slide.runToPos(Slide.HIGH_BASKET); speedMod = 0.25;}





    



        telemetry.addData("slide current", slide.slide.getCurrentPosition());
        telemetry.addData("speed mod: ", speedMod);
        telemetry.addData("slide target", target);
        telemetry.addData("claw pos", claw.claw.getPosition());
        telemetry.update();
    }
}
