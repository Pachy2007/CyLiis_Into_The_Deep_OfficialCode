package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class SampleAutoNodes {

    public static Pose2D putSamplePosition=new Pose2D (-730 , 180    ,-1);

    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{
            new Pose2D(-570 ,700 , -1.15) ,
            new Pose2D(-700 ,700 , -1.5) ,
            new Pose2D(-700 ,700 , -2.3)};

    public static Pose2D beforeTakeFromSub = new Pose2D(-60 , 1200 , -0.95);
    public static Pose2D prepareToTakeFromSubPosition = new Pose2D( 185 , 1400 , 0);
    public static Pose2D beforeGoPutSample= new Pose2D(0 , 1350 , -1);

    public static double[] takeFloorSampleExtendoPosition={175 , 150 , 260};


    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;


    public ElapsedTime timer;

    public boolean INIT=false;
    Node goPutSample, putSample,  takefromFloor, park , prepareToTakeFromSub , takeFromSub , takingFromSub , prepareGoPutSample;


    ElapsedTime timerTakeFloor = new ElapsedTime();
    ElapsedTime timerToWaitIntake = new ElapsedTime();
    ElapsedTime timerTryingToTake = new ElapsedTime();
    ElapsedTime timerVerifying = new ElapsedTime();

    public ElapsedTime timerToPark = new ElapsedTime();
    public Node currentNode;
    boolean skip=false;

    boolean a=false;

    SampleColor.State colorState;
    public void run(HardwareMap hardwareMap , Telemetry telemetry , SampleColor.State state)
    {

        if(!INIT)
        {
            colorState=state;
            timer=new ElapsedTime();
            goPutSample = new Node("goPutSample");
            putSample = new Node("putSample");
            takefromFloor = new Node("takefromFloor");
            park = new Node("park");
            prepareToTakeFromSub = new Node("prepareToTakeFromSub");
            takeFromSub = new Node("takeFromSub");
            takingFromSub = new Node("takingFromSub");
            prepareGoPutSample = new Node("prepareGoPutSample");

            Hardware.init(hardwareMap);
            Limelight.init(hardwareMap , 2);

            INIT=true;

            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake(SampleColor.State.YELLOW , false);
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            outtake.haveSample=true;

            ElapsedTime timer=new ElapsedTime();

            goPutSample.addConditions(
                    ()->{
                        timerToWaitIntake.reset();
                        intake.asure1inDepositState= Intake.Asure1inDeposit.Free;
                        if(outtake.state!= Outtake.State.Up)
                        Arm.putHighSample=0.5;
                        intake.setExtendoIN();intake.setState(Intake.State.INTAKE_UP);
                        if(intake.extendo.state== Extendo.State.IN && intake.ramp.state==intake.ramp.states.get("up"))outtake.grabSample();

                        outtake.goUp();
                        driveTrain.setTargetPosition(putSamplePosition);
                        Limelight.X=-1;
                        if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.UP)
                            Arm.putHighSample=0.74;
                    }
                    ,
                    ()->{
                        return (driveTrain.inPosition(25 , 25 , 0.3) && outtake.state==Outtake.State.Up);

                    }
                   ,
                   new Node[]{putSample}
            );
            putSample.addConditions(
                    ()->{

                        if( intake.ramp.state==intake.ramp.states.get("up"))
                        outtake.grabSample();


                        outtake.score();
                        intake.setState(Intake.State.REPAUS_DOWN);
                        timerTakeFloor.reset();
                        Limelight.X=-1;
                    }
                    ,
                    ()->{
                        timer.reset();
                        Limelight.X=-1;
                        return outtake.claw.state==outtake.claw.states.get("scoring");
                    }
                    ,
                    new Node[]{takefromFloor , takefromFloor , takefromFloor , prepareToTakeFromSub}
            );
            takefromFloor.addConditions(
                    ()->{
                        Limelight.X=-1;
                        outtake.score();
                        skip=false;
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takefromFloor.index]);
                        if(!intake.sampleInDeposit && driveTrain.inPosition(400 , 400 , 2))
                        intake.extendo.setTargetPosition(takeFloorSampleExtendoPosition[takefromFloor.index]);
                            if(!intake.sampleInDeposit)
                            intake.setState(Intake.State.INTAKE_DOWN);

                        if(intake.sampleInDeposit){intake.setExtendoIN();intake.setState(Intake.State.INTAKE_UP);}
                    }
                    ,
                    ()->{

                        if(!intake.sampleInDeposit && timerTakeFloor.seconds()>2)return true;
                        if(intake.sampleInDeposit && driveTrain.inPosition(50 , 50 , 0.2))
                         return true;

                             return false;
                    }
                    ,
                    new Node[]{goPutSample}
            );

            prepareToTakeFromSub.addConditions(
                    ()->{
                        outtake.score();
                        intake.setExtendoIN();
                        if(intake.state!= Intake.State.REPAUS_DOWN && intake.extendo.state== Extendo.State.IN && driveTrain.inPosition(30 , 30 , 0.15) && driveTrain.targetX==185){
                        intake.setState(Intake.State.REPAUS_DOWN);timerToWaitIntake.reset();}

                        if(intake.extendo.state!= Extendo.State.IN)intake.setState(Intake.State.REVERSE_UP);

                        driveTrain.setTargetPosition(prepareToTakeFromSubPosition);

                        if(Odo.getY()<870)driveTrain.setTargetPosition(beforeTakeFromSub);

                    }
                    ,
                    ()->{
                        if( driveTrain.inPosition(30 , 30 , 0.11) && timerToWaitIntake.seconds()>0.15 && intake.extendo.state== Extendo.State.IN && driveTrain.targetX==185 && intake.ramp.state==intake.ramp.states.get("down"))
                        {
                            if(timerTryingToTake.seconds()>0.5)
                            timerTryingToTake.reset();
                           if(timerTryingToTake.seconds()>0.35 && timerTryingToTake.seconds()<0.5)
                           Limelight.update();
                           if(Limelight.X!=-1)return true;
                        }
                        return false;
                    }
                    ,
                    new Node[]{takeFromSub}
            );
            takeFromSub.addConditions(
                    ()->{
                        if(MecanumDriveTrain.targetHeading==0)
                        {
                            driveTrain.setTargetPosition( Odo.getX() , Odo.getY() , -Odo.getHeading()+Limelight.targetAngle);
                            intake.setState(Intake.State.REPAUS_UP);
                        }
                        if(driveTrain.inPosition(35 , 35 , 0.11) && intake.ramp.state==intake.ramp.states.get("up"))
                        {
                            intake.setExtendoTargetPosition(Limelight.extendoPosition-20);
                        }
                    }
                    ,
                    ()->{
                        if(timerTryingToTake.seconds()>1.4)return true;
                        if(driveTrain.inPosition(35 , 35 , 0.11) && intake.extendo.state!= Extendo.State.IN && intake.extendo.inPosition())
                        {
                            timerTryingToTake.reset();
                            intake.setState(Intake.State.INTAKE_DOWN);
                            return intake.ramp.state==intake.ramp.states.get("down") && intake.extendo.inPosition();
                        }
                        return false;
                    }
                    ,
                    new Node[]{takingFromSub}
            );
            takingFromSub.addConditions(
                    ()->{
                        intake.setState(Intake.State.INTAKE_DOWN);
                        if(timerTryingToTake.seconds()>0.3)
                        intake.setExtendoTargetPosition(Limelight.extendoPosition+130);
                        else intake.setExtendoVelocity(0);
                    }
                    ,
                    ()->{
                        if(timerTryingToTake.seconds()>1.2)
                        {
                            Limelight.X=-1;
                            currentNode=prepareToTakeFromSub;
                        }
                        if(intake.sampleInDeposit && timerVerifying.seconds()>0.15)timerVerifying.reset();
                        if( intake.sampleInDeposit && (intake.sampleColor.state== SampleColor.State.YELLOW || intake.sampleColor.state==colorState) && timerVerifying.seconds()>0.1 && timerVerifying.seconds()<0.15 )
                        {
                            Limelight.X=-1;
                            return true;
                        }

                        return false;

                    }
                    ,
                    new Node[]{prepareGoPutSample}
            );
            prepareGoPutSample.addConditions(
                    ()->{
                        Limelight.X=-1;
                        if(intake.asure1inDepositState== Intake.Asure1inDeposit.Free)intake.asure1inDepositState= Intake.Asure1inDeposit.PrepareToClean;
                        if(intake.extendo.state== Extendo.State.GOING_IN)
                        driveTrain.setTargetPosition(beforeGoPutSample);
                    },
                    ()->{
                        return intake.extendo.state== Extendo.State.IN && intake.latch.state==intake.latch.states.get("open") && intake.asure1inDepositState== Intake.Asure1inDeposit.Done && driveTrain.inPosition(1000 , 1000 , 0.2);
                    }
                    ,
                    new Node[]{goPutSample}
            );
            park.addConditions(
                    ()->{
                        intake.setState(Intake.State.REVERSE_DOWN);
                        if(outtake.state!= Outtake.State.CLIMBAUTO)
                        outtake.autoCLimb();
                        driveTrain.setTargetPosition(prepareToTakeFromSubPosition);
                    }
                    ,
                    ()->{
                        return false;
                    }
                    ,
                    new Node[]{park}
            );

            currentNode=goPutSample;
        }

        else{
        currentNode.run();
        driveTrain.update();
        intake.update();
        outtake.update();
        Odo.update();
        if(timerToPark.seconds()>29 && currentNode!= goPutSample && currentNode!=putSample)currentNode=park;



        telemetry.addData("clawState" , outtake.claw.state.name);
        telemetry.addData("armState" , outtake.arm.state.name);
        telemetry.addData("liftState" , outtake.lift.state);
        telemetry.addData("outtakeSatte" , outtake.state);
        telemetry.addData("extension" , outtake.extension.servos[0].profile.getPosition());
        telemetry.addData("node" , currentNode.name);
        telemetry.update();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }

}
}
