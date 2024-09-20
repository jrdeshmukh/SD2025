package org.firstinspires.ftc.teamcode.wrappers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Claw {
    public Servo claw;

    public Claw(HardwareMap map) {
        claw = map.servo.get("claw");
    }

    public void setPos(double pos) {
        claw.setPosition(pos);
    }

    public class ClawClose implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            return false;
        }
    }
    public Action clawClose() {
        return new ClawClose();
    }
    public class ClawOpen implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            claw.setPosition(0);
            return false;
        }
    }
    public Action clawOpen() {
        return new ClawClose();
    }

}
