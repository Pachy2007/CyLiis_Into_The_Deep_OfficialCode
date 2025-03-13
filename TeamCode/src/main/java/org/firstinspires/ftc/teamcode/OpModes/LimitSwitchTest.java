package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "zzzz")
public class LimitSwitchTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);


        waitForStart();


        while(opModeIsActive())
        {
            telemetry.addData("LimitSwitchLift" , Hardware.liftLimitSwitch.getState());
            telemetry.addData("LimitSwitchExtendo" , Hardware.extendoBeamBreak.getState());
            telemetry.update();
        }
    }
}
