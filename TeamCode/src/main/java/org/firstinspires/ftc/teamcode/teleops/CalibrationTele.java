package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.CalibrateServo;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

@Config
@TeleOp()
public class CalibrationTele extends OpMode {
    MecanumDrive drive;
    Claw claw;
    Wrist wrist;
    Slide slide;

    CalibrateServo claw2, wrist2;
    BBG gp1, gp2;
    int target = 0;

    @Override
    public void init() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        wrist = new Wrist(hardwareMap);
        slide = new Slide(hardwareMap);
        claw = new Claw(hardwareMap);

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        claw2 = new CalibrateServo(claw.claw);
        wrist2 = new CalibrateServo(wrist.wrist);
        wrist.setPosition(0.5);
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x),
                -gamepad1.right_stick_x
        ));



        double clawPos = claw.claw.getPosition();
        double wristPos = wrist.wrist.getPosition();
        double increment = 0.03;

        if(gp2.dpad_down()) {
            wrist.setPosition(wristPos - increment);
        }
        if(gp2.dpad_up()) {
            wrist.setPosition(wristPos + increment);
        }

        if(gp2.a()) {
            claw.setPosition(clawPos - increment);
        }

        if (gp2.y()) {
            claw.setPosition(clawPos + increment);
        }


        slide.setPower(gamepad2.right_stick_y);



        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            target = Slide.LOW_BASKET;
        }

        telemetry.addData("Claw Pos: ", claw.claw.getPosition());
        telemetry.addData("Wrist Pos: ", wrist.wrist.getPosition());
        telemetry.addData("Slide Pos", slide.slide.getCurrentPosition());
        telemetry.addData("Slide power: ", gamepad2.right_stick_y);
        telemetry.update();



    }
}
