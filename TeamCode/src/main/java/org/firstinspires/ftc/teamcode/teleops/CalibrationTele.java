package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.wrappers.BBG;
import org.firstinspires.ftc.teamcode.wrappers.CalibrateServo;
import org.firstinspires.ftc.teamcode.wrappers.Claw;
import org.firstinspires.ftc.teamcode.wrappers.Slide;
import org.firstinspires.ftc.teamcode.wrappers.Wrist;

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

        gp1 = new BBG(gamepad1);
        gp2 = new BBG(gamepad2);

        claw2 = new CalibrateServo(claw.claw);
        wrist2 = new CalibrateServo(wrist.wrist);
    }

    @Override
    public void loop() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(
                gamepad1.left_stick_x,
                gamepad1.left_stick_y),
                gamepad1.right_stick_x
        ));

        double clawPos = claw.claw.getPosition();
        double wristPos = wrist.wrist.getPosition();
        double increment = 0.05;

        slide.setPower(gamepad2.right_stick_y*0.2);



        if (gamepad2.dpad_left || gamepad2.dpad_right) {
            target = Slide.LOW_BASKET;
        }

        telemetry.addData("Claw Pos: ", claw2.calibrate(gp2.dpad_up(), gp2.dpad_down()));
        telemetry.addData("Wrist Pos: ",wrist2.calibrate(gp2.y(), gp2.a()));
        telemetry.addData("Slide Pos", slide.slide.getCurrentPosition());
        telemetry.update();



    }
}
