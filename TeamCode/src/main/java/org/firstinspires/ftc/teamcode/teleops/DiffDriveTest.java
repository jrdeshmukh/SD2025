package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class DiffDriveTest extends OpMode
{

    Servo servoL, servoR; //ServoL is the one with the tag facing up
    double inc = 0.005;
    double dec = 0.33;


    @Override
    public void init() {
        servoL = hardwareMap.servo.get("servoL");
        servoR = hardwareMap.servo.get("servoR");
    }

    @Override
    public void loop() {
        if(gamepad1.dpad_down){
            servoL.setPosition(1);
            servoR.setPosition(0);
        }

        if(gamepad1.dpad_up){
            servoL.setPosition(0);
            servoR.setPosition(1);
        }

        if(Math.abs(gamepad1.left_trigger) > 0)
        {
            servoL.setPosition(servoL.getPosition() + (inc  * Math.abs(gamepad1.left_trigger)));
            servoR.setPosition(servoR.getPosition() + (inc * Math.abs(gamepad1.left_trigger)));
        }

        else if(Math.abs(gamepad1.right_trigger) > 0)
        {
            servoL.setPosition(servoL.getPosition() - (inc * Math.abs(gamepad1.right_trigger)));
            servoR.setPosition(servoR.getPosition() - (inc  * Math.abs(gamepad1.right_trigger)));
        }

        else{
            servoL.setPosition(servoL.getPosition() + (inc * gamepad1.right_stick_y));
            servoR.setPosition(servoR.getPosition() - (inc * gamepad1.right_stick_y));
        }

    }
}
