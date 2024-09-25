package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo wrist;

    public static double BOTTOM = 0.24, PICKUP = 0.33, DROP = 0.81, HIGH = 1;

    public Wrist(HardwareMap map) {
        wrist = map.servo.get("wrist");
    }

    public void setPos(double pos) {
        wrist.setPosition(pos);
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
            wrist.setPosition(0.33);
            return false;
        }
    }
    public Action wristPickup() {
        return new WristPickup();
    }

    public class WristDrop implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0.81);
            return false;
        }
    }
    public Action wristDrop() {
        return new WristDrop();
    }

    public class WristHigh implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(1);
            return false;
        }
    }
    public Action wristHigh() {
        return new WristHigh();
    }

}
