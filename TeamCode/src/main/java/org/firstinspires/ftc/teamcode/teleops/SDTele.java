package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

@TeleOp()
public class SDTele extends OpMode {
    MecanumDrive drive;
    Claw claw;
    Wrist wrist;
    Slide slide;

    int target = 0;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(hardwareMap);
        slide = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);
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

        if(gamepad2.right_bumper) {
            claw.setPos(Claw.OPEN);
        }

        if(gamepad2.left_bumper) {
            claw.setPos(Claw.CLOSE);
        }

        if (gamepad2.left_trigger>0.01) {
            wrist.setPos(Wrist.PICKUP);
        }

        if(gamepad2.right_trigger>0.01) {
            wrist.setPos(Wrist.DROP);
        }

        if(gamepad2.a) {
            wrist.setPos(Wrist.BOTTOM);
        }

        if(gamepad2.x || gamepad2.y || gamepad2.b) {
            wrist.setPos(Wrist.HIGH);
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
