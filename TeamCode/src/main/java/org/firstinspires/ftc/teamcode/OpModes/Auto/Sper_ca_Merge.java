package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Specimen7;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

        Specimen7 nodes=new Specimen7();


        Hardware.init(hardwareMap);




        nodes.run(hardwareMap , telemetry);

        while(opModeInInit())
        {
            nodes.intake.update();
            nodes.outtake.update();

            Hardware.IMUOFFSET=Math.PI;
            nodes.timerOuttake.reset();
        }
        waitForStart();
        while (opModeIsActive())
        {
            nodes.run(hardwareMap  , telemetry);

            telemetry.addData("extendo" , Limelight.extendoPosition);
            telemetry.addData("extendoPositionTarget" , Extendo.targetPosition);
            telemetry.update();

        }
    }
}
