package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Robot.Hardware;


@TeleOp
@Config
public class DifferentialTest extends LinearOpMode {


    public static double extendoPosition , liftPosition;

    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);


        Hardware.sch4.setPosition(0.7);
        Extendo extendo=new Extendo();
        Lift lift=new Lift();

        extendo.setIn();
        lift.goDown();

        while(opModeInInit())
        {
            extendo.update();
            lift.update();
        }

        waitForStart();

        while(opModeIsActive())
        {

        if(gamepad1.ps)
        {
            extendo.setIn();
            lift.goDown();
        }

        if(gamepad1.a || extendo.state== Extendo.State.OUT)
        {
            extendo.setTargetPosition(extendoPosition);
        }

        if(gamepad1.b){lift.goUp();
        lift.setPosition(liftPosition);}


        lift.update();
        extendo.update();

        telemetry.addData("encoderLift" , Hardware.mch0.getCurrentPosition());
        telemetry.addData("encoderExtendo" , Hardware.mch3.getCurrentPosition());

        telemetry.addData("LiftCurrent" , Hardware.mch0.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("extendoCurrent" , Hardware.mch3.getCurrent(CurrentUnit.AMPS));
        telemetry.update();

        }
    }
}
