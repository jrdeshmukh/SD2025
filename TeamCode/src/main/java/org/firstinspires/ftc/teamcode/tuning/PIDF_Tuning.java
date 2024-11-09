package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "PIDF_ARM_LOOP", group = "drive")
public class PIDF_Tuning extends OpMode {

    public PIDController controller;
    public static double p=0.015,i=0,d=0.0003,f=0.00005;
    public DcMotorEx motor;
    public final int MAX_POS = 3250;

    public final double ticksPerRotation = 288;

    public static int target = 0;

    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "slide");

        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
    public void loop() {
        controller.setPID(p,i,d);
        int pos = motor.getCurrentPosition();
        double pid = controller.calculate(pos, target);
        double ff = pos * f;
        double curPow = pid + ff;
        motor.setPower(curPow);
        telemetry.addData("power: ", curPow);
        telemetry.addData("pos: ", pos);
        telemetry.addData("lTarget: ", target);
        telemetry.update();
    }
}