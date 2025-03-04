package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.opencv.core.Mat;

@Autonomous
public class LimeLightContinous extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Odo.init(hardwareMap , telemetry);
        Odo.reset();
        Hardware.init(hardwareMap);

        Intake intake=new Intake(SampleColor.State.RED , false);
        Limelight.init(hardwareMap);
        MecanumDriveTrain mecanumDriveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);

        mecanumDriveTrain.setTargetPosition(0, 0 ,0);

        boolean prevX=false;
        boolean a=false;


        intake.setState(Intake.State.REPAUS_DOWN);
        waitForStart();

        while(opModeIsActive())
        {

            if(gamepad1.x && !prevX)
            {
                Limelight.update();

                mecanumDriveTrain.setTargetPosition(0 ,0 , -Odo.getHeading()+Math.atan(Limelight.X/Limelight.Y));
                intake.extendo.setIn();
                intake.setState(Intake.State.REPAUS_UP);
                intake.update();
                a=true;
            }
            /**if(mecanumDriveTrain.inPosition() && MecanumDriveTrain.targetHeading!=0)
            {if(intake.hasSample.getState()){
                if(intake.inPosition() && intake.ramp.state==intake.ramp.states.get("up"))
                intake.extendo.setTargetPosition(Limelight.extendoPosition);

                intake.extendo.update();
                if(intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN && intake.hasSample.getState() && intake.extendo.state!=Extendo.State.GOING_IN)intake.setState(Intake.State.INTAKE_DOWN);

                if(!intake.hasSample.getState()){if(intake.ramp.state==intake.ramp.states.get("up"))intake.setExtendoIN();}}

                if( !intake.hasSample.getState())
                {
                    intake.setState(Intake.State.INTAKE_UP);
                    intake.extendo.setIn();
                    mecanumDriveTrain.setTargetPosition(0 ,0 , Math.PI);
                    if(mecanumDriveTrain.inPosition())intake.setState(Intake.State.REVERSE_UP);
                }
                if(MecanumDriveTrain.targetHeading==Math.PI && intake.hasSample.getState())
                {
                    mecanumDriveTrain.setTargetPosition(0 ,0 ,0);
                    intake.setState(Intake.State.REPAUS_DOWN);
                    a=false;
                }
            }
            else if(a==false )intake.extendo.setIn();**/


            prevX=gamepad1.x;

            intake.update();
            Odo.update();
            mecanumDriveTrain.update();

        }
    }
}
