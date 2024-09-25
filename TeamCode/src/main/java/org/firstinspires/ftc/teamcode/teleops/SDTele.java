package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
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

import java.util.List;

@TeleOp()
public class SDTele extends OpMode {
    MecanumDrive drive;
    Wrist wrist;
    Servo claw2, wrist2;
    Slide slide;
    BBG gp1, gp2;
    //List<LynxModule> allHubs;

    int target = 0;

    @Override
    public void init() {
        //allHubs = hardwareMap.getAll(LynxModule.class);
        //for(LynxModule hub: allHubs) {
        //    hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        //}

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(hardwareMap);
        slide = new Slide(hardwareMap);
        claw2 = hardwareMap.get(Servo.class, "claw");
        wrist2 = hardwareMap.get(Servo.class, "wrist");
        gp2 = new BBG(gamepad2);
        gp1 = new BBG(gamepad1);
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x),
                -gamepad1.right_stick_x
        ));

        //wrist bottom 0.24, pickup 0.33, high1

        /**if(Math.abs(gamepad2.right_stick_y)>0) {
            slide.setPower(gamepad2.right_stick_y);
            target = slide.slide.getTargetPosition();
        }
        else {
            slide.setPosition(target);
        }**/

        slide.setPower(-gamepad2.left_stick_y);

        if(gp2.right_bumper()) {
            claw2.setPosition(1);
        }

        if(gp2.left_bumper()) {
            claw2.setPosition(Claw.CLOSE);
        }

        if (gamepad2.left_trigger>0.01) {
            wrist2.setPosition(Wrist.PICKUP);
        }

        if(gamepad2.right_trigger>0.01) {
            wrist2.setPosition(Wrist.DROP);
        }

        if(gp2.a()) {
            wrist2.setPosition(Wrist.BOTTOM);
        }

        if(gp2.x() || gp2.y() || gp2.b()) {
            wrist2.setPosition(Wrist.HIGH);
        }

        if(gamepad2.dpad_down) {
            target = Slide.BOTTOM;
        }

        if (gamepad2.dpad_up) {
            target = Slide.HIGH_BASKET;
        }

        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            target = Slide.LOW_BASKET;
        }



    }
}
