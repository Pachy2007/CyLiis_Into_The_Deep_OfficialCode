package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp(group = "z")
public class Climb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);

        boolean a=true;
        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.a && gamepad1.dpad_up)a=true;
            if(gamepad1.b && gamepad1.dpad_up) a=false;




        }
    }
}
