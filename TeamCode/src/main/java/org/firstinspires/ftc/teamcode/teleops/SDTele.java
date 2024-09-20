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
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y),
                gamepad1.right_stick_x
        ));

        if(Math.abs(gamepad2.right_stick_y)>0) {
            slide.setPower(gamepad2.right_stick_y);
            target = slide.slide.getTargetPosition();
        }
        else {
            slide.setPosition(target);
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
