package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slide {
    public DcMotorEx slide;
    private final PIDController controller;
    public final double p = 0.008, i = 0, d = 0.0005, f = 0.00005;
    public static int offset = 0;

    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 3150;
    public static final int LOW_BASKET = 1875;
    public static final int HIGH_RUNG = 1013;
    public boolean active = false;
    public static double curPow = 0;
    public double targetPosition = 0;
    public static boolean humanControl = false;

    public Slide(HardwareMap map) {
        slide = map.get(DcMotorEx.class, "slide");
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
    }

    public void runToPos(int targetPosition) {
        this.targetPosition = targetPosition;
        humanControl = false;
    }

    public void setPow2() {
        int pos = slide.getCurrentPosition();
        double ff = pos * f;
        if(humanControl) {
            slide.setPower(ff);
            return;
        }
        double pid = controller.calculate(pos, targetPosition);
        curPow = pid + ff;
        slide.setPower(curPow);
    }

    public class ResetEncoder implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            targetPosition = 0;
            return false;
        }
    }

    public Action resetEncoders() {
        return new ResetEncoder();
    }



    public class SetPow implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            int pos = slide.getCurrentPosition();
            double pid = controller.calculate(pos, targetPosition);
            double ff = pos * f;
            curPow = pid + ff;
            slide.setPower(curPow);
            return true;
        }
    };

    public Action setPow() {
        return new SetPow();
    }

    public class HighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            targetPosition = Slide.HIGH_RUNG;
            return false;
        }
    }

    public Action highRung() {
        return new HighRung();
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


    public class WaitToDone implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            return Math.abs(slide.getCurrentPosition()-targetPosition)<50;
        }
    }

    public Action waitToDone() {
        return new WaitToDone();
    }

    public void setPower(double power) {
        slide.setPower(power);
    }

    public Action autoMove(int pos) {
        return new InstantAction(() -> runToPos(pos));
    }


}
