package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw;

    public static final double OPEN = 1, CLOSE = 0.67;

    public Claw(HardwareMap map) {
        claw = map.servo.get("claw");
    }

    public void setPosition(double pos) {
        claw.setPosition(pos);
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0.39);
            return false;
        }
    }
    public Action close() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0.81);
            return false;
        }
    }

    public Action open() {
        return new ClawOpen();
    }

}
