package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {
    public Servo wrist;

    public Wrist(HardwareMap map) {
        wrist = map.servo.get("claw");
    }

    public void setPos(double pos) {
        wrist.setPosition(pos);
    }

    public class WristClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0);
            return false;
        }
    }
    public Action WristClose() {
        return new WristClose();
    }
    public class WristOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            wrist.setPosition(0);
            return false;
        }
    }
    public Action clawOpen() {
        return new WristOpen();
    }

}
