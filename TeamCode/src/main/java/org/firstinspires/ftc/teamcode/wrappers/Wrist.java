package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo wrist;

    public static double BOTTOM = 0.24, PICKUP = 0.1494, DROP = 0.7489, HIGH = 0.86;
    public static double HIGH_RUNG = 0.527;

    public Wrist(HardwareMap map) {
        wrist = map.servo.get("wrist");
    }

    public void setPos(double pos) {
        wrist.setPosition(pos);
    }


    public class WristHighRung implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(HIGH_RUNG);
            return false;
        }
    }

    public Action highRung() {
        return new WristHighRung();
    }

    public class WristBottom implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.24);
            return false;
        }
    }
    public Action wristBottom() {
        return new WristBottom();
    }

    public class WristPickup implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(PICKUP);
            return false;
        }
    }
    public Action wristPickup() {
        return new WristPickup();
    }

    public class WristDrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(DROP);
            return false;
        }
    }
    public Action wristDrop() {
        return new WristDrop();
    }

    public class WristHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(HIGH);
            return false;
        }
    }
    public Action wristHigh() {
        return new WristHigh();
    }

}
