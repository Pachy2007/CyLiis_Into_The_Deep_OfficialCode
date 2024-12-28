package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Auto.AutoNodes;
import org.firstinspires.ftc.teamcode.Auto.SampleAutoNodes;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@Autonomous
public class Sper_ca_Merge_Sample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

         SampleAutoNodes nodes=new SampleAutoNodes();

        Hardware.init(hardwareMap);

        DigitalChannel bb=hardwareMap.get(DigitalChannel.class , "bb");
        Intake intake=new Intake();
        Outtake outtake=new Outtake(Outtake.State.DeafultWithElement);
        Extendo extendo=new Extendo();
        Latch latch=new Latch();

        outtake.haveSample=true;


        while(opModeInInit())
        {
            intake.update();
            outtake.update();
            extendo.update();
            latch.update();
        }
        waitForStart();
        while (opModeIsActive())
        {
            Odo.update();
            nodes.run(hardwareMap  , telemetry);


            telemetry.addData("x" , nodes.driveTrain.controllerX.calculate( nodes.driveTrain.targetX, Odo.getX()));
            telemetry.addData("y" , nodes.driveTrain.controllerY.calculate( nodes.driveTrain.targetY, Odo.getY()));

            telemetry.addData("x1" , nodes.driveTrain.targetX);
            telemetry.addData("y1" , nodes.driveTrain.targetY);

            telemetry.addData("error" , MecanumDriveTrain.realHeading-MecanumDriveTrain.targetHeading);
            telemetry.addData("outtakeState" , nodes.outtake.state.name());
            telemetry.addData("State" , nodes.currentNode.name);
            telemetry.addData("X" , Odo.getX());
            telemetry.addData("Y" , Odo.getY());
            telemetry.addData("heading" , Odo.getHeading());
            telemetry.update();
        }
    }
}
