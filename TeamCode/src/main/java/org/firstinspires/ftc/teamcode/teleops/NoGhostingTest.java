package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp()
public class NoGhostingTest extends OpMode {

    DcMotorEx fl, fr, bl, br, slide;

    @Override
    public void init() {
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {
        fl.setPower(gamepad1.left_stick_y);
        bl.setPower(gamepad1.left_stick_y);
        fr.setPower(gamepad1.left_stick_y);
        br.setPower(gamepad1.left_stick_y);

        slide.setPower(-gamepad2.left_stick_y);
    }
}
