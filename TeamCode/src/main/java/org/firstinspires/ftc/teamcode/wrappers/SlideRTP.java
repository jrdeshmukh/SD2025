package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SlideRTP {
    public DcMotorEx slide;
    private final double p = 0, i = 0, d = 0, f = 0;

    private static final double TICKS_IN_DEGREE = 5;
    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;

    public SlideRTP(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setTargetPosition(0);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setPower(double power) {
        if(Math.abs(power) > 0) {
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            slide.setPower(power);
        }
    }

    public void setPosition(int position) {
        if(!slide.isBusy()) {
            slide.setTargetPosition(position);
            slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slide.setPower(1);
        }

    }

}
