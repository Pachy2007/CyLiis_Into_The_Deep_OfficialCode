package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Color;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.opencv.core.Mat;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Z")
public class TeleOpWithSensors extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Extendo extendo=new Extendo();
        Intake intake=new Intake();
        Latch latch=new Latch();

        Outtake.prim=false;
        boolean take=false;


         boolean a=false;
         org.firstinspires.ftc.teamcode.Climb.Climb climb=new Climb();
        boolean redAll=true;

        DigitalChannel bb;

        Color blueTarget = new Color(0, 0 ,255);
        Color redTarget = new Color(255, 0, 0);
        Color yellowTarget = new Color(255, 255, 0);

        bb=hardwareMap.get(DigitalChannel.class , "bb");

        while(opModeInInit())
        {
            outtake.update();
            intake.update();
            extendo.update();
            driveTrain.update();
            latch.update();
        }
        waitForStart();

        while(opModeIsActive()){

            double X=gamepad1.left_stick_x;
            double Y=-gamepad1.left_stick_y;
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );

            if(gamepad1.a && gamepad1.left_bumper)a=true;
            if(gamepad1.b && gamepad1.left_bumper) a=false;

            if(a==false){
                if(gamepad1.right_bumper) climb.goUp();
                else if(gamepad1.left_bumper)climb.goDown();
                else climb.Default();}
            else climb.Constant();

            double power=-gamepad1.right_stick_y;
            extendo.setVelocity(power);
            if(gamepad1.a)extendo.setIn();


            if(gamepad1.options) Odo.reset();

            if(gamepad2.y)outtake.goDefault();

            if((gamepad2.a || (take==true && extendo.state==Extendo.State.IN)) && outtake.state== Outtake.State.Deafult)
            {
                take=false;
                outtake.grabSample();
            }

            if(gamepad1.circle && outtake.state==Outtake.State.Specimen)outtake.grabSample();




            if(gamepad2.dpad_up && outtake.state==Outtake.State.DeafultWithElement)outtake.goUp();

            if(gamepad1.circle && outtake.state==Outtake.State.DeafultWithElement)
            {
                outtake.releaseSample();
            }

            if(gamepad1.circle)outtake.score();

            if(gamepad2.dpad_left)outtake.goForLow();
            if(gamepad2.dpad_right)outtake.goForHigh();

            if(gamepad2.circle && outtake.state== Outtake.State.Deafult)
            {
                outtake.takeSpecimen();
            }

            if(gamepad2.right_trigger>0)intake.setState(Intake.State.INTAKE_DOWN);
            else if(gamepad2.left_trigger>0)intake.setState(Intake.State.REVERSE_DOWN);
            else if(extendo.state== Extendo.State.GOING_IN || Math.abs(power)>0.1)intake.setState(Intake.State.INTAKE_UP);
            else intake.setState(Intake.State.REPAUS_UP);


            if(!bb.getState() && outtake.state== Outtake.State.Deafult)
            {
                take=true;
                extendo.setIn();intake.setState(Intake.State.INTAKE_UP);
                latch.setState("goOpen");
            }
            else {if(outtake.state!= Outtake.State.Deafult)take=false;}

            if(bb.getState() && outtake.state!=Outtake.State.TakingElement)latch.setState("goClose");
            else latch.setState("goOpen");





            telemetry.addData("extendo" , extendo.state);
            telemetry.addData("outtake" , outtake.state);
            telemetry.addData("intake" , intake.state);
            telemetry.addData("latch" , latch.state.name);
            telemetry.addData("claw" , outtake.claw.state.name);
            telemetry.addData("haveSample" , outtake.haveSample);
            telemetry.addData("heading",Odo.getHeading());



            telemetry.update();
            outtake.update();
            extendo.update();
            intake.update();
            latch.update();
            Odo.update();
        }
    }
}