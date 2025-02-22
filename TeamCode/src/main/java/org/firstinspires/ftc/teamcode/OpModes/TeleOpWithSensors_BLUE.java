package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Intake.ExtendoBlocker;
import org.firstinspires.ftc.teamcode.Modules.Intake.Latch;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "a")
public class TeleOpWithSensors_BLUE extends LinearOpMode {



    public static double val=0.1;
    public static double revereTime=0.3;

    public static SampleColor.State prevState=SampleColor.State.YELLOW;

    @Override
    public void runOpMode() throws InterruptedException {



        if(Odo.INIT)
            Odo.odo.setPosition(new Pose2D(DistanceUnit.MM , 0 ,0 , AngleUnit.RADIANS , Odo.getHeading()-Hardware.IMUOFFSET));

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);
        ElapsedTime verifyChangeColor=new ElapsedTime();
        ElapsedTime reverseTimer=new ElapsedTime();
        ElapsedTime bbtimer=new ElapsedTime();

        boolean reverse=false;
        boolean prevBB=true;

        ElapsedTime timer=new ElapsedTime();
        DigitalChannel bbGuide=hardwareMap.get(DigitalChannel.class , "bbGuide");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());



        SampleColor color=new SampleColor();
        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Extendo extendo=new Extendo();
        Intake intake=new Intake(SampleColor.State.BLUE , true);
        Latch latch=new Latch();
        ExtendoBlocker extendoBlocker=new ExtendoBlocker();

        ElapsedTime climbTimer=new ElapsedTime();

        boolean prevPs=false;
        PTO pto=new PTO();

        boolean a=false;

        boolean enableClimb=false;

        Wheelie wheelie=new Wheelie();




        Outtake.prim=false;
        boolean take=false;
        boolean BB=true;

        DigitalChannel bb;

        bb=hardwareMap.get(DigitalChannel.class , "bb");

        outtake.extension.setState("goRetrect");
        outtake.arm.setState("goHighSpecimen");

        while(opModeInInit())
        {
            outtake.extension.update();
            outtake.arm.update();
            intake.update();
            extendo.update();
            latch.update();
            extendoBlocker.update();
        }
        waitForStart();

        while(opModeIsActive()){


            if(!enableClimb){
                if(gamepad1.right_bumper){wheelie.goDown();wheelie.up=false;wheelie.goDown=true;}
                else if(gamepad1.left_bumper){wheelie.goDown();wheelie.up=true;wheelie.goDown=true;}
                else wheelie.keepUp();}

            if(gamepad1.ps && !prevPs)
            {
                if(pto.state==pto.states.get("normal"))pto.setState("climb");
                else if(pto.state==pto.states.get("climb"))pto.setState("normal");
            }
            prevPs=gamepad1.ps;
            if(gamepad1.dpad_up)
            {
                a=true;
                Differential.liftPower=1;
                Differential.update();
            }
            else if(gamepad1.dpad_down)
            {
                a=true;
                Differential.liftPower=-1;
                Differential.update();
            }
            else a=false;




            double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );





            double power=-gamepad1.right_stick_y;
            if(!take)
            {
                if(power!=0 || extendo.state!= Extendo.State.IN)extendoBlocker.setState("goOpen");

                if(extendoBlocker.state==extendoBlocker.states.get("open"))
                    extendo.setVelocity(power);
                extendo.update();
            }
            if(gamepad1.a)extendo.setIn();


            if(gamepad1.options) Odo.reset();

            if(gamepad2.y && outtake.state!= Outtake.State.Up && outtake.state!=Outtake.State.TakingElement)outtake.goDefault();

            if((gamepad2.a || (take==true && extendo.state==Extendo.State.IN && timer.seconds()>0.35 && intake.ramp.state==intake.ramp.states.get("up"))) && outtake.state== Outtake.State.Deafult && outtake.inPosition())
            {
                take=false;
                outtake.grabSample();
            }

            if((gamepad1.circle) && outtake.state==Outtake.State.Specimen)outtake.grabSample();



            if(gamepad2.x)outtake.retry();

            if(gamepad2.dpad_up && outtake.state==Outtake.State.DeafultWithElement)outtake.goUp();

            if((gamepad1.circle) && (outtake.state==Outtake.State.DeafultWithElement || outtake.state== Outtake.State.ReleaseSample) && outtake.haveSample==true)
            {
                outtake.releaseSample();
            }

            if(gamepad1.circle)outtake.score();

            if(gamepad2.dpad_left)outtake.goForLow();
            if(gamepad2.dpad_right)outtake.goForHigh();

            if(gamepad2.circle)
            {
                outtake.takeSpecimen();
            }

            if(!BB && prevBB)bbtimer.reset();

            if((!take && outtake.state!=Outtake.State.TakingElement) && ( (color.state!= SampleColor.State.RED && !BB && reverseTimer.seconds()>revereTime) || BB))
            {if(gamepad2.right_trigger>0)intake.setState(Intake.State.INTAKE_DOWN);
            else if(gamepad2.left_trigger>0){intake.setState(Intake.State.REVERSE_DOWN);            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;}
            else if(extendo.state== Extendo.State.GOING_IN || Math.abs(power)>0.1)intake.setState(Intake.State.INTAKE_UP);
            else intake.setState(Intake.State.REPAUS_UP);}



            if(!BB && outtake.state== Outtake.State.Deafult && outtake.inPosition() && !take && color.state!= SampleColor.State.RED && bbtimer.seconds()>val && reverse)
            {
                if(color.state== SampleColor.State.RED)return;
                take=true;
                extendo.setIn();
                latch.setState("goOpen");
                intake.setState(Intake.State.INTAKE_UP);
                timer.reset();
            }
            else {if(outtake.state!= Outtake.State.Deafult){take=false;if(timer.seconds()>0.25)latch.setState("goClose");}}




            if(latch.inPosition() && take && timer.seconds()<0.15)intake.setState(Intake.State.INTAKE_UP);
            if(latch.inPosition() && take && timer.seconds()>0.15 && timer.seconds()<0.3){intake.setState(Intake.State.REVERSE_UP);ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;}
            if(take && timer.seconds()>0.3){intake.setState(Intake.State.INTAKE_UP);latch.setState("goClose");}

            if(timer.seconds()>0.3)latch.setState("goClose");



            if(take && verifyChangeColor.seconds()>0.05 && color.state== SampleColor.State.RED)take=false;

            if(!BB){if(color.state== SampleColor.State.RED && bbtimer.seconds()>val)reverseTimer.reset();}

            if(reverseTimer.seconds()<revereTime && color.state== SampleColor.State.RED)
            {            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;
                intake.setState(Intake.State.REVERSE_DOWN);reverse=false;}
            else reverse=true;



            if(extendo.state== Extendo.State.IN && power==0)extendoBlocker.setState("goClose");

            prevBB=BB;
            BB=bb.getState();


            prevState=color.state;
            color.update();


            extendoBlocker.update();
            wheelie.update();
            if(!a)
                outtake.update();
            extendo.update();
            intake.update();
            latch.update();
            Odo.update();
            pto.update();


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
            telemetry.addData("heading" , Odo.getHeading());
            telemetry.addData("climbTimer" , climbTimer.seconds());
            telemetry.addData("IMUOFFSET" , Hardware.IMUOFFSET);




            telemetry.update();

            if(color.state!=prevState)verifyChangeColor.reset();

        }
    }
}