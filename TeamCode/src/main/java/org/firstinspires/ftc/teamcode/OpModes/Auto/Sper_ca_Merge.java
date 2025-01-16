package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Auto.SpecimenAutoNodes;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

        SpecimenAutoNodes nodes=new SpecimenAutoNodes();


        Hardware.init(hardwareMap);




        nodes.run(hardwareMap , telemetry);

        while(opModeInInit())
        {
            nodes.intake.update();
            nodes.outtake.update();
            nodes.extendo.update();
            nodes.latch.update();
            Hardware.IMUOFFSET=Math.PI;
            nodes.timerOuttake.reset();
        }
        waitForStart();
        while (opModeIsActive())
        {
            nodes.run(hardwareMap  , telemetry);

        }
    }
}
