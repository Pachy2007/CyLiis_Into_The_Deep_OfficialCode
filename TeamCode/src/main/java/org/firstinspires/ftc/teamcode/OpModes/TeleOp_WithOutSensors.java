package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Others.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "a")
public class TeleOp_WithOutSensors extends LinearOpMode {



    public static double val=0.1;
    public static double revereTime=0.3;

    public static SampleColor.State prevState=SampleColor.State.YELLOW;

    @Override
    public void runOpMode() throws InterruptedException {



        ElapsedTime verifyChangeColor=new ElapsedTime();
        ElapsedTime reverseTimer=new ElapsedTime();
        ElapsedTime bbtimer=new ElapsedTime();

        boolean reverse=false;
        boolean prevBB=true;

        ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;

        ElapsedTime timer=new ElapsedTime();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        SampleColor color=new SampleColor();
        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Extendo extendo=new Extendo();
        Intake intake=new Intake();
        Latch latch=new Latch();


        Outtake.prim=false;
        boolean take=false;
        boolean BB=true;

        boolean a=false;
        Climb climb=new Climb();
        boolean redAll=true;

        DigitalChannel bb;

        bb=hardwareMap.get(DigitalChannel.class , "bb");

        while(opModeInInit())
        {
            outtake.update();
            intake.update();
            extendo.update();
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
            if(!take)
                extendo.setVelocity(power);
            if(gamepad1.a)extendo.setIn();


            if(gamepad1.ps) Odo.reset();

            if(gamepad2.y)outtake.goDefault();

            if((gamepad2.a || (take==true && extendo.state==Extendo.State.IN && timer.seconds()>0.2 && intake.ramp.state==intake.ramp.states.get("up"))) && outtake.state== Outtake.State.Deafult && outtake.inPosition())
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

            if(!BB && prevBB)bbtimer.reset();

            if((!take && outtake.state!=Outtake.State.TakingElement) && ( (color.state!= SampleColor.State.RED && !BB && reverseTimer.seconds()>revereTime) || BB))
                if(gamepad2.right_trigger>0)intake.setState(Intake.State.INTAKE_DOWN);
                else if(gamepad2.left_trigger>0){intake.setState(Intake.State.REVERSE_DOWN);            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;}
                else if(extendo.state== Extendo.State.GOING_IN || Math.abs(power)>0.1)intake.setState(Intake.State.INTAKE_UP);
                else intake.setState(Intake.State.REPAUS_UP);



            if(!BB && outtake.state== Outtake.State.Deafult && outtake.inPosition() && !take && color.state!= SampleColor.State.RED && bbtimer.seconds()>val && reverse)
            {
                if(color.state== SampleColor.State.RED)return;
                take=true;
                extendo.setIn();
                latch.setState("goOpen");
                intake.setState(Intake.State.INTAKE_UP);
                timer.reset();
            }
            else {if(outtake.state!= Outtake.State.Deafult){take=false; if(timer.seconds()>0.15)latch.setState("goClose");}}




            if(latch.inPosition() && take && timer.seconds()<0.06)intake.setState(Intake.State.INTAKE_UP);
            if(latch.inPosition() && take && timer.seconds()>0.06 && timer.seconds()<0.15){intake.setState(Intake.State.REVERSE_UP);            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;}
            if(take && timer.seconds()>0.15){intake.setState(Intake.State.INTAKE_UP);latch.setState("goClose");}

            if(timer.seconds()>0.15)latch.setState("goClose");



            if(take && verifyChangeColor.seconds()>0.05 && color.state== SampleColor.State.RED)take=false;

            if(!BB){if(color.state== SampleColor.State.RED && bbtimer.seconds()>val)reverseTimer.reset();}

            if(reverseTimer.seconds()<revereTime && color.state== SampleColor.State.RED)
            {            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;
                intake.setState(Intake.State.REVERSE_DOWN);reverse=false;}
            else reverse=true;



            BB=false;




            outtake.update();
            extendo.update();
            intake.update();
            latch.update();
            Odo.update();


            telemetry.addData("bbTimer" , bbtimer.seconds());
            telemetry.addData("reverseTimer" , reverseTimer.seconds());
            telemetry.addData("IntakeState" , intake.state);
            telemetry.addData("take" , take);
            telemetry.addData("timer" , timer.seconds());
            telemetry.addData("BB" , !BB);
            telemetry.addData("prevBB" , !prevBB);
            telemetry.addData("color" , color.state);
            telemetry.addData("claw" , outtake.claw.state.name);
            telemetry.addData("arm" , outtake.arm.state.name);




            telemetry.update();

            if(color.state!=prevState)verifyChangeColor.reset();

        }
    }
}