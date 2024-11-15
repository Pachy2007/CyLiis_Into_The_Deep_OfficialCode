package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp
public class Climb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        org.firstinspires.ftc.teamcode.Climb.Climb climb=new org.firstinspires.ftc.teamcode.Climb.Climb();

        waitForStart();

        while(opModeIsActive())
        {
            if(gamepad1.dpad_up) climb.goUp();
            else if(gamepad1.dpad_down)climb.goDown();
            else climb.Default();
        }
    }
}
