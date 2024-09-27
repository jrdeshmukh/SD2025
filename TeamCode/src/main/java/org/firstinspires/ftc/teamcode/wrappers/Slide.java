package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    public DcMotorEx slide;
    private final PIDController controller;
    public final double p = 0, i = 0, d = 0, f = 0;

    private static final double TICKS_IN_DEGREE = 384.5 / 360;
    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;
    public boolean active = false;
    public static double curPow = 0;
    public static double targetPosition = 0;

    public Slide(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
    }

    public void runToPos(int targetPosition) {
        int pos = slide.getCurrentPosition();
        double pid = controller.calculate(pos, targetPosition);
        double ff = targetPosition * f;
        curPow = pid + ff;
        slide.setPower(curPow);
    }



    public class SetPow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int pos = slide.getCurrentPosition();
            double pid = controller.calculate(pos, targetPosition);
            double ff = targetPosition * f;
            curPow = pid + ff;
            slide.setPower(curPow);
            return true;
        }
    };

    public Action setPow() {
        return new SetPow();
    }



    public class LiftHigh implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = Slide.HIGH_BASKET;
            return false;
        }
    }


    public Action liftHigh() {
        return new LiftHigh();
    }

    public class LiftLow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
             targetPosition = Slide.LOW_BASKET;
             return false;
        }
    }
    public Action liftLow() {
        return new LiftLow();
    }

    public class LiftBottom implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = Slide.BOTTOM;
            return false;
        }
    }
    public Action liftBottom() {
        return new LiftBottom();
    }

    public void setPower(double power) {
        slide.setPower(power);
    }


}
