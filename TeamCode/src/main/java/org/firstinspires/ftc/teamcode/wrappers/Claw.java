package org.firstinspires.ftc.teamcode.wrappers;

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

}
