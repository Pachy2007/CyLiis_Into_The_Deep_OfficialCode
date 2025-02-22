package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.SampleAutoNodes;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge_Sample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

         SampleAutoNodes nodes=new SampleAutoNodes();

        Hardware.init(hardwareMap);
        nodes.run(hardwareMap  , telemetry);

        while(opModeInInit())
        {
            nodes.intake.update();
            nodes.outtake.update();
            nodes.extendo.update();
            nodes.latch.update();
            Hardware.IMUOFFSET=-Math.PI/2;
            Arm.withElementSample=0.27;
        }
        waitForStart();
        while (opModeIsActive())
        {
            Arm.withElementSample=0.045;
            Odo.update();
            nodes.run(hardwareMap  , telemetry);
        }
    }
}
