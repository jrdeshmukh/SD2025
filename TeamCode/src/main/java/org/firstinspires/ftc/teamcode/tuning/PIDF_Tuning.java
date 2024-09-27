package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name = "PIDF_ARM_LOOP", group = "drive")
public class PIDF_Tuning extends OpMode {

    public PIDController controller;
    public static double p=0,i=0,d=0,f=0;
    public DcMotorEx motor;
    public final int MAX_POS = 200;

    public final double ticksPerRotation = 288;

    public static int target = 0;

    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor = hardwareMap.get(DcMotorEx.class, "slide");

    }
    public void loop() {
        if(target > MAX_POS)
            target = MAX_POS;

        controller.setPID(p,i,d);
        int pos = motor.getCurrentPosition();

        double pid = controller.calculate(pos, target);

        double ff = target * f;


        double power = pid + ff;

        motor.setPower(power);
        telemetry.addData("pos: ", pos);
        telemetry.addData("lTarget: ", target);
        telemetry.update();
    }
}