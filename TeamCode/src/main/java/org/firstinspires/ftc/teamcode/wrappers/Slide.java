package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    private DcMotorEx slide;
    private PIDController controller;
    private double p = 0, i = 0, d = 0, f = 0;

    private static final double TICKS_IN_DEGREE = 5;
    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;

    public Slide(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        controller = new PIDController(p, i, d);
    }

    public boolean setPosition(int targetPosition) {
        int pos = slide.getCurrentPosition();
        double pid = controller.calculate(pos, targetPosition);
        double ff = Math.cos(Math.toRadians(targetPosition / TICKS_IN_DEGREE)) * f;
        slide.setPower(pid + ff);
        return (Math.abs(pid) > 0.05);
    }

    public class LiftHigh implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return setPosition(Slide.HIGH_BASKET);
        }
    }
    public Action liftHigh() {
        return new LiftHigh();
    }

    public class LiftLow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return setPosition(Slide.LOW_BASKET);
        }
    }
    public Action liftLow() {
        return new LiftLow();
    }

    public class LiftBottom implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return setPosition(Slide.BOTTOM);
        }
    }
    public Action liftBottom() {
        return new LiftBottom();
    }

    public void setPower(double power) {
        slide.setPower(power);
    }

    public void setPowerPos(double power, int pos) {
        if(Math.abs(power)>0) setPower(power);
        else setPosition(pos);
    }

}
