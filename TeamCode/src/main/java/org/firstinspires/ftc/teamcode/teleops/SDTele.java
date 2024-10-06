package org.firstinspires.ftc.teamcode.teleops;

import  com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

import java.util.List;

@TeleOp()
public class SDTele extends OpMode {
    MecanumDrive drive;
    Servo claw, wrist;
    Slide slide;
    BBG gp1, gp2;
    List<LynxModule> allHubs;

    int target = 20;
    double curpow = 0;
    double speedMod = 0.75;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        slide = new Slide(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
    }

    @Override
    public void loop() {
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
        if (gp2.x())                                   wrist.setPosition(0.4383); //sstraight out
        if (gp2.y())                                   wrist.setPosition(Wrist.DROP); //score, 60 degree above flat

        if (gamepad2.dpad_down)                        slide.runToPos(Slide.BOTTOM);
        if (gamepad2.dpad_up)                          slide.runToPos(Slide.HIGH_BASKET);
        if (gamepad2.dpad_left || gamepad2.dpad_right) target = Slide.LOW_BASKET;

        telemetry.addData("slide current", slide.slide.getCurrentPosition());
        telemetry.addData("speed mod: ", speedMod);
        telemetry.addData("slide target", target);
        telemetry.addData("claw pos", claw.getPosition());
        telemetry.update();
    }
}
