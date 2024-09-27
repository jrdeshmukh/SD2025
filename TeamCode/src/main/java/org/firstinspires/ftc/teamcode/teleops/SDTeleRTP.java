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
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.SlideRTP;

import java.util.List;

@TeleOp()
public class SDTeleRTP extends OpMode {
    MecanumDrive drive;
    Servo claw, wrist;
    SlideRTP slide;
    BBG gp1, gp2;
    List<LynxModule> allHubs;

    int target = 0;

    @Override
    public void init() {
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub: allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        slide = new SlideRTP(hardwareMap);

        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
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

        if (gp2.right_bumper())                        claw.setPosition(1.00);
        if (gp2.left_bumper())                         claw.setPosition(0.67);

        if (gamepad2.left_trigger>0.01)                wrist.setPosition(0.33);
        if (gamepad2.right_trigger>0.01)               wrist.setPosition(0.81);
        if (gp2.a())                                   wrist.setPosition(0.24);
        if (gp2.x() || gp2.y() || gp2.b())             wrist.setPosition(1);

        if (gamepad2.dpad_down)                        target = Slide.BOTTOM;
        if (gamepad2.dpad_up)                          target = Slide.HIGH_BASKET;
        if (gamepad2.dpad_left || gamepad2.dpad_right) target = Slide.LOW_BASKET;

        if(Math.abs(gamepad2.right_stick_y)>0) {
            slide.setPower(-gamepad2.left_stick_y);
            target = slide.slide.getTargetPosition();
        }
        else {
            slide.setPosition(target);
        }

        slide.setPower(-gamepad2.left_stick_y);


    }
}
