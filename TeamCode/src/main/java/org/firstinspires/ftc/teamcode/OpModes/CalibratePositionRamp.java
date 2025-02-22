package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Ramp;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;

@TeleOp
public class CalibratePositionRamp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        Extendo extendo=new Extendo();
        Intake intake=new Intake(SampleColor.State.RED , true);

        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.x)extendo.setIn();
            telemetry.addData("extendoPosition" , Extendo.position);
            telemetry.addData("rampPosition" , Ramp.upPosition);
            telemetry.update();
            extendo.update();
            intake.update();
        }
    }
}
