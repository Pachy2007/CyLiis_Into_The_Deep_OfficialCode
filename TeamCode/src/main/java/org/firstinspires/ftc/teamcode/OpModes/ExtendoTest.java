package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@Config
@TeleOp
public class ExtendoTest extends LinearOpMode {
    public static double position;
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);


        Extendo extendo=new Extendo();
        extendo.setTargetPosition(0);
        waitForStart();
        while (opModeIsActive())
        {
            extendo.setTargetPosition(position);
            extendo.update();
            telemetry.addData( "position", extendo.motor.getPosition());
            telemetry.update();
        }
    }
}
