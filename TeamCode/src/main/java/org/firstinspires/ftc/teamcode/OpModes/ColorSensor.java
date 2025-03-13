package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "zzzz")
public class ColorSensor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);
        SampleColor sampleColor=new SampleColor();


        waitForStart();

        while (opModeIsActive())
        {
            sampleColor.update();


            telemetry.addData("color" , sampleColor.state);
            telemetry.addData("red" , sampleColor.red);
            telemetry.addData("green" , sampleColor.green);
            telemetry.addData("blue" , sampleColor.blue);


            telemetry.update();
        }
    }
}
