package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import  com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
    Servo claw, wrist;
    Slide slide;
    BBG gp1, gp2;
    List<LynxModule> allHubs;
    Pose2d basket;
    FtcDashboard dash = FtcDashboard.getInstance();
    Claw clawA;
    Wrist wristA;
    Action goAction;

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
        clawA = new Claw(hardwareMap);
        wristA = new Wrist(hardwareMap);
        goAction = drive.actionBuilder(drive.pose).strafeToLinearHeading(basket.position, basket.heading).build();

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
    }

    @Override
    public void loop() {
        if(gamepad2.left_stick_button) {
            runningActions.add(new SequentialAction(
                    goAction,
                    wristA.wristHigh(),
                    slide.liftHigh(),
                    wristA.wristDrop(),
                    clawA.open()
            ));
        }

        TelemetryPacket packet = new TelemetryPacket();
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;

        dash.sendTelemetryPacket(packet);
        if(gamepad1.right_trigger>0.03) speedMod = 1;
        if(gamepad1.right_bumper) speedMod = 0.75;
        if(gamepad1.left_trigger>0.03) speedMod = 0.25;
        if(gamepad1.left_bumper) speedMod = 0.5;


        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                -gamepad1.left_stick_y*speedMod,
                -gamepad1.left_stick_x*speedMod),
                -gamepad1.right_stick_x*speedMod
        ));
        drive.updatePoseEstimate();

        if(Math.abs(gamepad2.left_stick_y)>0) {
            telemetry.addData("pp", 1);
            slide.setPower(gamepad2.left_stick_y);
            target = slide.slide.getCurrentPosition();
        }
        else {
            curpow = slide.runToPos(target);
        }


        if (gp2.right_bumper())                        claw.setPosition(0.81); //open
        if (gp2.left_bumper())                         claw.setPosition(0.39); //close

        if (gamepad2.left_trigger>0.01)                wrist.setPosition(0.86); //straight up
        if (gamepad2.right_trigger>0.01)               wrist.setPosition(0.1494); //pickup
        if (gp2.x())                                   wrist.setPosition(0.5089); //sstraight out
        if (gp2.y())                                   wrist.setPosition(0.7489); //score, 60 degree above flat

        if (gamepad2.dpad_down)                        target = Slide.BOTTOM;
        if (gamepad2.dpad_up)                          target = Slide.HIGH_BASKET;
        if (gamepad2.dpad_left || gamepad2.dpad_right) target = Slide.LOW_BASKET;


        if(gamepad1.a) {
            basket = drive.pose;
            goAction = drive.actionBuilder(drive.pose).strafeToLinearHeading(basket.position, basket.heading).build();

        }






        telemetry.addData("slide current", slide.slide.getCurrentPosition());
        telemetry.addData("speed mod: ", speedMod);
        telemetry.addData("slide target", target);
        telemetry.addData("slide power: ", curpow);
    }
}
