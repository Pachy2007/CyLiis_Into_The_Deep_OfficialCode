package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.BetterSpecimenAuto;
import org.firstinspires.ftc.teamcode.Auto.SpecimenAuto;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge_Spec_BLUE extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

        BetterSpecimenAuto nodes=new BetterSpecimenAuto();


        Hardware.init(hardwareMap);
        nodes.init(hardwareMap , SampleColor.State.BLUE, 3);

        nodes.intake.extendo.setIn();

        while(opModeInInit())
        {
            nodes.intake.update();
            nodes.outtake.update();

        }
        waitForStart();
        while (opModeIsActive())
        {
            nodes.run(telemetry);
            telemetry.addData("nodeState" , nodes.currentNode.name);
            telemetry.addData("inatkeState" , nodes.intake.state);
            telemetry.addData("extendoState" , nodes.intake.extendo.state);
            telemetry.update();

        }
    }
}
