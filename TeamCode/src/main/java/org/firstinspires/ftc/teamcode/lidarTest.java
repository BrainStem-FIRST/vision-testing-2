package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "Lidar Testing")
public class lidarTest extends LinearOpMode {

    AnalogInput ranger;
    AnalogInput ranger2;

    @Override
    public void runOpMode() throws InterruptedException {

        ranger = hardwareMap.get(AnalogInput.class, "ranger");
        ranger2 = hardwareMap.get(AnalogInput.class, "ranger2");

        waitForStart();

        while (opModeIsActive()) {
//            telemetry.addData("Raw Voltage", ranger.getVoltage());
            //telemetry.addData("20-deg mode", (ranger.getVoltage()*48.7)-4.9);
            telemetry.addData("15-deg mode FRONT", (ranger.getVoltage()*32.5)-2.6);
            telemetry.addData("15-deg mode SIDE", (ranger2.getVoltage()*32.5)-2.6);
            telemetry.update();

        }
    }
}