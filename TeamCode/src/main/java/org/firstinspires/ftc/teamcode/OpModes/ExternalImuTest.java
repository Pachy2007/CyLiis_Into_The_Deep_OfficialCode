package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "zzz")
@Config
public class ExternalImuTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);



        waitForStart();

        while(opModeIsActive())
        {
            telemetry.update();
        }
    }
}
