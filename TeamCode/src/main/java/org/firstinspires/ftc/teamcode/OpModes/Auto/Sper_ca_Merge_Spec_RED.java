package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.BetterSpecimenAuto;
import org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge_Spec_RED extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

        BetterSpecimenAuto nodes=new BetterSpecimenAuto();


        Hardware.init(hardwareMap);
        nodes.init(hardwareMap , SampleColor.State.RED , 0);
        nodes.outtake.usingSpecialPosition=false;
        nodes.intake.extendo.setIn();

        while(opModeInInit())
        {
            nodes.intake.update();
            nodes.outtake.update();
            Odo.update();
        }
        waitForStart();
        while (opModeIsActive())
        {
            nodes.run(telemetry);
        }
    }
}