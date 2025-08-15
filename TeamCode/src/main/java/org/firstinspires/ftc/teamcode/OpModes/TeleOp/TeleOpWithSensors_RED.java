package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "pachy")
public class TeleOpWithSensors_RED extends LinearOpMode {

    enum State{
        CONTROLLING , CLIMB
    }
    State state=State.CONTROLLING;
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean inTeleop=false;
        boolean outtakeReady=false;
        double initialX=0 , initialY=0;

        boolean prevCircle=false;

        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Intake intake=new Intake(SampleColor.State.RED , true);

        PTO pto=new PTO();
        Wheelie wheelie=new Wheelie();
        boolean climb=false;
        boolean prevA=false;


        Node prepareClimbLvl2 , climbLvl2 , prepareClimbLvl3 , climbLvl3 , readyToBeStopped;
        Node climbState;

        ElapsedTime timer=new ElapsedTime();
        ElapsedTime timerSpec=new ElapsedTime();

        prepareClimbLvl2 = new Node("prepareClimbLvl2");
        climbLvl2 = new Node("climbLvl2");
        prepareClimbLvl3 = new Node("prepareClimbLvl3");
        climbLvl3 = new Node("climbLvl3");
        readyToBeStopped = new Node("readyToBeStopped");
        ElapsedTime readyToBeStoppedTimer=new ElapsedTime();
        prepareClimbLvl2.addConditions(
                ()->{

                    wheelie.goUp();
                    outtake.climb2();
                }
                ,
                ()->{
                    if(wheelie.inPosition() && outtake.inPosition()) {
                        //Lift.treshold=-1;
                        return true;}
                    return false;
                }
                ,
                new Node[]{climbLvl2}
        );

        climbLvl2.addConditions(
                ()->{
                    if(outtake.lift.state!= Lift.State.DOWN)
                        pto.setState("goClimb");
                    else pto.setState("goNormal");
                    outtake.goDefault();
                    //intake.extendo.setTargetPosition(350);
                    /*if(gamepad1.dpad_left)
                    {
                        Lift.treshold=60;
                    }*/
                }
                ,
                ()->{
                    if(Lift.State.DOWN==outtake.lift.state && readyToBeStoppedTimer.seconds()>1.5){wheelie.goDown();readyToBeStoppedTimer.reset();
                    }
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN && readyToBeStoppedTimer.seconds()>0.3 && readyToBeStoppedTimer.seconds()<1.5)
                    {readyToBeStoppedTimer.reset();return true;}
                    return false;
                }
                ,
                new Node[]{prepareClimbLvl3}
        );
        prepareClimbLvl3.addConditions(
                ()->{
                    if(readyToBeStoppedTimer.seconds()<2 && outtake.lift.encoder.getPosition()>130)
                        pto.setState("goNormal");
                    if(readyToBeStoppedTimer.seconds()>2)
                        pto.setState("goClimb");
                    if(!outtake.lift.inPosition() && outtake.lift.encoder.getPosition()<900)
                        intake.extendo.setTargetPosition(500);
                    else intake.extendo.setIn();
                    if(outtake.lift.encoder.getPosition()>150)
                        wheelie.goUp();
                    if(outtake.arm.state!=outtake.arm.states.get("goFinalClimb3") || outtake.arm.state!=outtake.arm.states.get("finalClimb3"))
                        outtake.arm.setState("climb");
                    if(readyToBeStoppedTimer.seconds()>0.15)
                        outtake.climb3();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state!= Lift.State.DOWN)
                    {
                        outtake.arm.setState("goFinalClimb3");
                    }
                    if(outtake.lift.inPosition())intake.extendo.setIn();
                    return  outtake.lift.inPosition() && outtake.lift.state!= Lift.State.DOWN && intake.extendo.state== Extendo.State.IN && intake.extendoBlocker.state==intake.extendoBlocker.states.get("close");
                }
                ,
                new Node[]{climbLvl3}
        );
        climbLvl3.addConditions(
                ()->{
                    pto.setState("goClimb");
                    if(intake.extendo.state== Extendo.State.IN)
                        outtake.goDefault();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN)
                    {
                        Differential.liftPower=-1;
                        readyToBeStoppedTimer.reset(); return true;}
                    return false;
                }
                ,
                new Node[]{readyToBeStopped}
        );
        readyToBeStopped.addConditions(
                ()->{
                    outtake.climb=true;
                    Differential.liftPower=-1;
                    if(readyToBeStoppedTimer.seconds()>0.05)
                        wheelie.goDown();


                }
                ,()->{
                    return false;
                }
                ,
                new Node[]{readyToBeStopped}
        );
        climbState=prepareClimbLvl2;

        while(opModeInInit())
        {
            gamepad1.setLedColor(44, 128, 14 , 2100000000);
        }
        waitForStart();

        while(opModeIsActive()){



            if(gamepad1.dpad_up){
                state=State.CLIMB;
            }

            if(state==State.CONTROLLING)
            {
                readyToBeStoppedTimer.reset();
                driveTrain.setMode(MecanumDriveTrain.State.DRIVE);
                double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
                double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
                double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

                double heading =-Odo.getHeading();

                double x=X*Math.cos(heading)-Y*Math.sin(heading);
                double y=X* Math.sin(heading)+Y*Math.cos(heading);

                if( inTeleop && (Math.sqrt( (initialY-Odo.getY())*(initialY-Odo.getY()) + (initialX-Odo.getX())*(initialX-Odo.getX()) )>200 || gamepad1.triangle))outtakeReady=true;

                if((Math.abs(x)>0.1 || Math.abs(y)>0.1 || Math.abs(rotation)>0.1) && !inTeleop)
                {inTeleop=true;
                 initialY=Odo.getY();
                 initialX=Odo.getX();}
                driveTrain.setTargetVector( x , y , rotation );

                if(gamepad1.options)Odo.reset();

                if(inTeleop){
                if(gamepad1.a && !prevA)
                {   if(intake.extendo.state!= Extendo.State.IN)
                    intake.setExtendoIN();
                else intake.extendo.setTargetPosition(800);}
                prevA=gamepad1.a;


                if(gamepad2.y && outtake.state!=Outtake.State.TakingElement)outtake.goDefault();

                if(((gamepad2.a || (intake.stateTransfer== Intake.TransferLogic.ReadyToTransfer && intake.extendo.state== Extendo.State.IN))) && intake.ramp.state==intake.ramp.states.get("up") && intake.transfer && outtake.inPosition() && outtake.claw.state==outtake.claw.states.get("open")){outtake.grabSample();timer.reset();
                }

                if(!Intake.bbDeposit.getState() && intake.justColorAllaiance && intake.transfer && outtake.state== Outtake.State.Specimen && intake.stateTransfer==Intake.TransferLogic.Transfer)outtake.state= Outtake.State.Deafult;
                if(gamepad1.circle && outtake.state==Outtake.State.Specimen && !prevCircle)outtake.grabSample();
                prevCircle=gamepad1.circle;
                if(gamepad2.x)outtake.retry(); //failsafePusSpecimen

                if(gamepad2.dpad_up)
                {outtake.goUp();outtake.goForHigh();}
                if(gamepad2.dpad_down){outtake.goUp();outtake.goForLow();}


                if(!intake.transfer && outtake.state== Outtake.State.Deafult && intake.stateTransfer==Intake.TransferLogic.Free)outtake.goDefault();

                if(!gamepad1.circle)timerSpec.reset();
                if(timerSpec.seconds()>0.3)outtake.state= Outtake.State.Deafult;

                if((gamepad1.circle) && (outtake.state==Outtake.State.DeafultWithElement || outtake.state== Outtake.State.ReleaseSample) && outtake.haveSample==true)
                {
                    outtake.releaseSample();
                }
                if(gamepad1.circle)outtake.score();
                if(gamepad2.circle)outtake.takeSpecimen();

                if(gamepad2.left_bumper){intake.justColorAllaiance=false;intake.transfer=true; outtake.onSpec=false;outtake.goDefault();}
                if(gamepad2.right_bumper){intake.justColorAllaiance=true;intake.transfer=false; outtake.onSpec=true;outtake.goDefault();}
                if(gamepad2.touchpad){intake.justColorAllaiance=true; intake.transfer=true; outtake.onSpec=true;outtake.goDefault();}

                if(gamepad1.right_bumper)intake.setState(Intake.State.INTAKE_DOWN);
                else if(gamepad1.left_bumper){intake.setState(Intake.State.REVERSE_DOWN);ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;}
                else intake.setState(Intake.State.REPAUS_UP);

                double power=-gamepad1.right_stick_y;
                intake.setExtendoVelocity(power);

            }}

            if(state==State.CLIMB)
            {
                if(gamepad1.dpad_left)
                {
                    Lift.treshold=60;
                }

                if(gamepad1.dpad_down){Differential.liftPower=0;climb=true;}
                else if(!climb){
                    climbState.run();
                    if(climbState.transition())climbState=climbState.next[Math.min(climbState.index++ , climbState.next.length-1)];}
            }


            if(outtake.state== Outtake.State.Up && Lift.position==Outtake.highBasketPosition)
            {
                Hardware.meh0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Hardware.mch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Hardware.meh1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                Hardware.mch0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            }
            else{
                Hardware.meh0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Hardware.mch1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Hardware.meh1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                Hardware.mch0.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }


            driveTrain.update();
            Odo.update();
            if(inTeleop){
                if(outtakeReady)
            outtake.update();
                else {
                    outtake.arm.setState("goNeutralSpecimen");
                    outtake.arm.update();
                        outtake.extension.update();
                        outtake.claw.update();}
            intake.update();
            pto.update();
            wheelie.update();}

            telemetry.addData("outtake state" , outtake.state);
            telemetry.addData("extension state" , outtake.extension.state.name);
            telemetry.addData("arm state" , outtake.arm.state.name);
            telemetry.addData("claw state" , outtake.claw.state.name);
            telemetry.addData("intake" , intake.state);
            telemetry.addData("transfer" , intake.stateTransfer);
            telemetry.update();

        }
    }
}

