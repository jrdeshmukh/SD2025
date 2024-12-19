package org.firstinspires.ftc.teamcode.wrappers;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Spin {
    public Servo spin;
    public static final double SPIN_0 = 0.8;

    public Spin(HardwareMap map) {
        spin = map.servo.get("spin");
        spin.setPosition(SPIN_0);
    }

    public void setSpin(double pos) {
        double actualPos = pos;
        if(pos < 0) {
            actualPos = pos + 1;
        }
        else if(pos>1){
            actualPos = pos - 1;
        }
        spin.setPosition(actualPos);
    }

    public Action spin0() {
        return new InstantAction(() -> spin.setPosition(SPIN_0));
    }
}
