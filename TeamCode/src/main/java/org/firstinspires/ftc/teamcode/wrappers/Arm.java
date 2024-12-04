package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Arm {
    public DcMotorEx arm;
    private final PIDController controller;
    public final double p = 0.00, i = 0, d = 0.000;
    public static int offset = 0;

    public static final int BOTTOM = 0;
    public static final int HIGH_BASKET = 0;
    public static final int LOW_BASKET = 0;
    public static final int HIGH_RUNG = 0;
    public boolean active = false;
    public static double curPow = 0;
    public double targetPosition = 0;
    public static boolean humanControl = false;
    public static double TICKS_PER_DEGREE = 200;

    public Arm(HardwareMap map) {
        arm = map.get(DcMotorEx.class, "slide");
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        controller = new PIDController(p, i, d);
        controller.setPID(p, i, d);
    }

    public void runToPos(int targetPosition) {
        this.targetPosition = targetPosition;
        humanControl = false;
    }

    public double getAngle() {
        return arm.getCurrentPosition()/TICKS_PER_DEGREE;
    }

    public void setPow2() {
        int pos = arm.getCurrentPosition();
        if(humanControl) {
            return;
        }
        curPow = controller.calculate(pos, targetPosition);
        arm.setPower(curPow);
    }

    public class ResetEncoder implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            int pos = arm.getCurrentPosition();
            double pid = controller.calculate(pos, targetPosition);
            arm.setPower(pid);
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

    public void setPower(double power) {
        arm.setPower(power);
    }


}
