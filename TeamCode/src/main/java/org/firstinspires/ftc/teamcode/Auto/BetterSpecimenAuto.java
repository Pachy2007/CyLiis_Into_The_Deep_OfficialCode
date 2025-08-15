package org.firstinspires.ftc.teamcode.Auto;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class BetterSpecimenAuto {

    public static Pose2D[] beforePutSpecimenPosition = {
            new Pose2D(780 ,110 ,0 ) ,
            new Pose2D(700 ,60 ,-0 ) ,
            new Pose2D ( 880 ,-90 ,-0.33)
    };
    public static Pose2D[] putSpecimenPosition = {
            new Pose2D ( 780 , 110 ,0) ,
            new Pose2D ( 790 , 60 ,0) ,
            new Pose2D (880 ,-90 ,-0.35) ,
            new Pose2D (880 ,-90 ,-0.35) ,
            new Pose2D (880 ,-90 ,-0.35) ,
            new Pose2D (880 ,-90 ,-0.35) ,
            new Pose2D (880 ,-90 ,-0.35) ,
    };

    public static Pose2D[] releaseSamplePosition = {
            new Pose2D(150 ,-720 ,-0) ,
            new Pose2D(250 ,-700 ,-0.9)

    };

    public static Pose2D[] takeFloorSamplePosition = {
            new Pose2D(500 ,-850 ,0.39),
            new Pose2D(500 ,-850 ,0.82),
            new Pose2D(500 ,-900,0.96)
    };

    public static Pose2D throwPosition = new Pose2D(500, -850 ,2.3);

    public static Pose2D beforeTakeWallSpecimenPosition[] ={
            new Pose2D ( 125 , -720 ,0) ,
            new Pose2D ( 110 , -720 ,0) ,
            new Pose2D( 55 ,-720 ,-0.7) ,
            new Pose2D( 55 ,-720 ,-0.7) ,
            new Pose2D( 55 ,-720, -0.7)
    };

    public static Pose2D[] takeWallSpecimenPosition = {
            new Pose2D(75 ,-710 ,0) ,
            new Pose2D(0 ,-710 ,0) ,
            new Pose2D(10 ,-710 ,0),
            new Pose2D(10 ,-710 ,0),
            new Pose2D(10 ,-710 ,0),

    };

    public static Pose2D[] afterTakeWallSpecimenPosition={
            new Pose2D( 150 , -600 , -0.68)
    };

    public static double releaseSampleExtendoPosition=750;
    public static double[] extendoPositionPreextend={200, 200 , 200 , 0};
    public static double[] takeFromFloorEstendoPosition={430 ,610 ,840 , 0};
    public static double throwExtendoPosition = 360;
    boolean tookFromFLoor=false;

    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    ElapsedTime timerVerifying=new ElapsedTime();
    ElapsedTime timerToWait =new ElapsedTime();
    ElapsedTime timerTryingToTake = new ElapsedTime();
    ElapsedTime timerWaitingTryAgain = new ElapsedTime();
    ElapsedTime timerToReverse = new ElapsedTime();
    ElapsedTime timerDropDown = new ElapsedTime();
    Node beforePutSpecimen , putSpecimen , prepareToTakeFromSub , takeFromSub , takeAgain , prepareReleaseSampleFromSub,releaseSampleFromSub , takeFloorSample , releaseSample , throwNode , beforeTakeWallSpecimen , takeWallSpecimen , afterTakeWallSpecimen;
    public Node currentNode;

    SampleColor.State state;

    public void init(HardwareMap hardwareMap , SampleColor.State state , int pipeline)
    {

        this.state=state;
        Hardware.init(hardwareMap);
        Limelight.init(hardwareMap , pipeline);
        Limelight.targetAngle=0;

        beforeTakeWallSpecimen = new Node("beforeTakeWallSpecimen");
        beforePutSpecimen = new Node("beforePutSpecimen");
        putSpecimen = new Node("putSpecimen");
        takeFloorSample = new Node("takeFloorSample");
        releaseSample = new Node("releaseSample");
        takeWallSpecimen = new Node("takeWallSpecimen");
        prepareToTakeFromSub = new Node("prepareToTakeFromSub");
        takeFromSub = new Node("try6");
        releaseSampleFromSub = new Node("releaseSampleFromSub");
        takeAgain = new Node("takeAgain");
        prepareReleaseSampleFromSub = new Node("prepareReleaseSampleFromSub");
        afterTakeWallSpecimen = new Node("afterTakeWallSpecimen");
        throwNode = new Node("throwNode");

        intake = new Intake(SampleColor.State.RED , false);
        intake.state= Intake.State.REPAUS_DOWN;
        outtake = new Outtake(Outtake.State.DeafultWithElement);
        outtake.haveSample = false;
        outtake.usingSpecialPosition=false;
        driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.PID);


        beforePutSpecimen.addConditions(
                ()->{
                    if(intake.sampleInDeposit)intake.setState(Intake.State.REVERSE_UP);
                    outtake.goUp();
                    driveTrain.setTargetPosition( beforePutSpecimenPosition[Math.min(beforePutSpecimenPosition.length-1 , beforePutSpecimen.index)]);
                    if(beforePutSpecimen.index==0 || beforePutSpecimen.index==1)
                    intake.setState(Intake.State.REPAUS_DOWN);
                }
                ,
                ()->{
                    return (driveTrain.inPosition(450 , 450 , 1) || (Odo.xVelocity<30 && Odo.getX()>700)) && outtake.inPosition() && (outtake.lift.state== Lift.State.GOING_UP || outtake.lift.state== Lift.State.UP);
                }
                ,
                new Node[]{putSpecimen}
        );

        putSpecimen.addConditions(
                ()->{
                    outtake.usingSpecialPosition=false;
                    if(intake.sampleInDeposit)intake.setState(Intake.State.REVERSE_UP);
                    driveTrain.setTargetPosition(putSpecimenPosition[Math.min(putSpecimenPosition.length-1 , putSpecimen.index)]);
                    if(driveTrain.inPosition(40 , 40 , 0.05) || (driveTrain.inPosition(180 , 330 , 0.4) && putSpecimen.index>1))
                    {outtake.score();outtake.takeSpecimen();}
                    intake.asure1inDepositState= Intake.Asure1inDeposit.Free;
                }
                ,
                ()->{
                    if(putSpecimen.index<2 && driveTrain.inPosition(30 , 30 , 0.05))
                    Limelight.update();
                    timerTryingToTake.reset();
                    prepareToTakeFromSub.index=0;
                    takeFromSub.index=0;
                    takeAgain.index=0;
                    if(( outtake.state!= Outtake.State.Up && putSpecimen.index>1) ||(driveTrain.inPosition(10 , 20 , 0.2)) || (Math.abs(Odo.xVelocity)<2 && Odo.getX()>700))
                    {
                        outtake.score();
                        outtake.takeSpecimen();
                        if(outtake.state== Outtake.State.Specimen)
                    return true;
                    }
                    return false;
                }
                ,
                new Node[]{prepareToTakeFromSub , prepareToTakeFromSub , beforeTakeWallSpecimen}

        );

        prepareToTakeFromSub.addConditions(
                ()->{
                    intake.transfer=false;
                    if(outtake.inPosition())outtake.goDefault();
                    if(MecanumDriveTrain.targetHeading==0)Limelight.update();

                    if(Limelight.extendoPosition!=-1)
                    {if(intake.extendo.state== Extendo.State.IN)
                    intake.setState(Intake.State.REPAUS_UP);}
                }
                ,
                ()->{
                    if(intake.ramp.state==intake.ramp.states.get("up") && MecanumDriveTrain.targetHeading==0 && Limelight.extendoPosition!=-1) {
                        driveTrain.setTargetPosition(Odo.x, Odo.y, -Odo.heading + Limelight.targetAngle);
                        timerTryingToTake.reset();
                    }


                    if(timerTryingToTake.seconds()>2)
                    {
                        intake.setExtendoIN();
                        if(!tookFromFLoor)
                        prepareToTakeFromSub.next[0]=takeFloorSample;
                        else prepareToTakeFromSub.next[0]=beforeTakeWallSpecimen;
                        return true;
                    }
                    if(driveTrain.inPosition(1000 , 1000 , 0.08) && MecanumDriveTrain.targetHeading!=0 && timerDropDown.seconds()>0.5 && intake.state== Intake.State.REPAUS_UP)
                    intake.setExtendoTargetPosition(Limelight.extendoPosition-20);
                    if( intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN && intake.ramp.state==intake.ramp.states.get("down") )
                    {
                        timerTryingToTake.reset();
                        return true;
                    }
                    if(intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN && timerDropDown.seconds()>0.5){timerDropDown.reset();   intake.setState(Intake.State.INTAKE_DOWN);intake.setExtendoVelocity(0);}
                    return false;
                }
                ,
                new Node[]{takeFromSub}
        );

        takeFromSub.addConditions(
                ()->{
                    if(outtake.inPosition())outtake.goDefault();
                    if(intake.asure1inDepositState== Intake.Asure1inDeposit.Free)
                    {
                        if(timerDropDown.seconds()>0.7)
                        intake.setExtendoTargetPosition(Limelight.extendoPosition+80);
                        if(!intake.sampleInDeposit)
                    intake.setState(Intake.State.INTAKE_DOWN);}
                }
                ,
                ()->{
                    if(timerTryingToTake.seconds()>1.5)
                    {
                        takeFromSub.next[0]=takeAgain;
                        if(!tookFromFLoor && takeFromSub.index==1)
                        takeFromSub.next[1]=takeFloorSample;
                        else takeFromSub.next[1]=beforeTakeWallSpecimen;
                        intake.extendo.setIn(); intake.setState(Intake.State.REVERSE_UP);
                        timerWaitingTryAgain.reset();
                        return true;
                    }
                    else{
                        takeFromSub.next[0]=prepareReleaseSampleFromSub;
                        takeFromSub.next[1]=prepareReleaseSampleFromSub;
                    }

                    if(intake.sampleInDeposit && timerVerifying.seconds()>0.17)
                    {timerVerifying.reset();intake.setState(Intake.State.INTAKE_UP);}
                    if(intake.sampleInDeposit && this.state==intake.sampleColor.state && timerVerifying.seconds()>0.07 && timerVerifying.seconds()<0.17){
                        intake.asure1inDepositState= Intake.Asure1inDeposit.PrepareToClean;
                        takeFromSub.next[takeFromSub.index]=takeFromSub.next[0]=prepareReleaseSampleFromSub;
                        takeFromSub.next[takeFromSub.index]=takeFromSub.next[1]=prepareReleaseSampleFromSub;
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{prepareReleaseSampleFromSub , prepareReleaseSampleFromSub}
        );

        takeAgain.addConditions(
                ()->{
                    if(intake.driverState!= Intake.State.REPAUS_DOWN && intake.state!= Intake.State.REPAUS_DOWN)
                    {intake.setState(Intake.State.REVERSE_UP);timerToWait.reset();}
                    intake.setExtendoIN();
                    driveTrain.setTargetPosition(Odo.x , Odo.y , 0);
                    timerTryingToTake.reset();
                    Limelight.extendoPosition=-1;
                    MecanumDriveTrain.targetHeading=0;
                }
                ,
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)intake.setState(Intake.State.REPAUS_DOWN);
                    return driveTrain.inPosition() && intake.ramp.state==intake.ramp.states.get("down") && intake.state== Intake.State.REPAUS_DOWN && timerToWait.seconds()>0.1;
                }
                ,
                new Node[]{prepareToTakeFromSub}
        );

        prepareReleaseSampleFromSub.addConditions(
                ()->{
                    if(prepareReleaseSampleFromSub.index==0 && takeAgain.index==0)
                    outtake.usingSpecialPosition=true;
                    if(intake.extendo.state== Extendo.State.GOING_IN && MecanumDriveTrain.targetHeading==-0.9-Math.floor((-0.9/ (Math.PI*2)))*Math.PI*2 && driveTrain.inPosition(1000 , 1000 , 0.55))
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , prepareReleaseSampleFromSub.index)]);
                    else if(intake.extendo.encoder.getPosition()<300)driveTrain.setTargetPosition(Odo.getX() , Odo.getY() , -0.9);
                }
                ,
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)intake.asure1inDepositState= Intake.Asure1inDeposit.Free;

                    return ((driveTrain.inPosition(350 , 350 , 0.55))) && MecanumDriveTrain.targetHeading==-0.9-Math.floor((-0.9/ (Math.PI*2)))*Math.PI*2;
                }
                ,
                new Node[]{releaseSample}
        );

        releaseSample.addConditions(
                ()->{
                    if(outtake.state== Outtake.State.Deafult && intake.extendo.state== Extendo.State.IN)
                    {outtake.grabSample();}
                    if(outtake.extension.state==outtake.extension.states.get("retrect") && outtake.state== Outtake.State.DeafultWithElement)
                    {
                        intake.setState(Intake.State.REPAUS_UP);
                        outtake.releaseSample();
                    }
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , releaseSample.index)]);
                        if(intake.extendo.state== Extendo.State.IN)intake.asure1inDepositState= Intake.Asure1inDeposit.Free;

                    if(driveTrain.inPosition(450 , 450  , 0.4) && outtake.state== Outtake.State.ReleaseSample && (outtake.arm.servos[1].getPosition()>0.9)){outtake.takeSpecimen();}
                }
                ,
                ()->{
                    if(tookFromFLoor)releaseSample.next[0]=beforeTakeWallSpecimen;
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.5)timerToReverse.reset();
                    if(outtake.state== Outtake.State.Specimen){
                        timerTryingToTake.reset(); intake.setExtendoIN(); return true;}
                    return false;
                }
                ,
                new Node[]{beforeTakeWallSpecimen , takeFloorSample}

        );

        takeFloorSample.addConditions(
                ()->{
                    outtake.takeSpecimen();
                    ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;
                    tookFromFLoor=true;
                    driveTrain.setTargetPosition(takeFloorSamplePosition[Math.min(takeFloorSamplePosition.length-1 , takeFloorSample.index)]);

                    if(intake.extendo.encoder.getPosition()>50 && intake.extendo.state!= Extendo.State.GOING_IN)intake.setState(Intake.State.INTAKE_DOWN);
                    else if(takeFloorSample.index==0)
                    {
                        if(intake.sampleInDeposit)
                        intake.setState(Intake.State.REVERSE_UP);
                        else intake.setState(Intake.State.REPAUS_DOWN);}

                    if(!driveTrain.inPosition(95 , 95 , 0.2) && intake.extendo.state!= Extendo.State.GOING_IN){timerTryingToTake.reset();}
                    else if(driveTrain.inPosition(95 , 95 , 0.2)){
                        intake.setExtendoTargetPosition(takeFromFloorEstendoPosition[takeFloorSample.index]);
                        if(intake.extendo.encoder.getPosition()>50 && intake.extendo.state!= Extendo.State.GOING_IN)
                        intake.setState(Intake.State.INTAKE_DOWN);
                    }

                }
                ,
                ()->{
                    if(takeFloorSample.next[takeFloorSample.index]==beforeTakeWallSpecimen)return true;

                        return (intake.sampleInDeposit || timerTryingToTake.seconds()>1) && driveTrain.inPosition(105 , 105 , 0.42);
                }
                ,
                new Node[]{throwNode , throwNode , throwNode , beforeTakeWallSpecimen}
        );

        throwNode.addConditions(
                ()->{
                    driveTrain.setTargetPosition(throwPosition);
                    intake.setExtendoTargetPosition(throwExtendoPosition);

                    if(driveTrain.inPosition(1000 , 1000 , 1.3))intake.setState(Intake.State.REVERSE_UP);
                }
                ,
                ()->{
                    timerTryingToTake.reset();
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.3 && driveTrain.inPosition(1000 , 1000 ,1.3))timerToReverse.reset();
                    if(timerToReverse.seconds()>0.1 && timerToReverse.seconds()<0.3 && driveTrain.inPosition(1000 , 1000 , 1.3))
                    {intake.setExtendoTargetPosition(extendoPositionPreextend[throwNode.index+1]);return true;}
                    return false;
                }
                ,
                new Node[]{takeFloorSample , takeFloorSample , beforeTakeWallSpecimen}
        );

        beforeTakeWallSpecimen.addConditions(
                ()->{
                    intake.setExtendoIN();
                    outtake.takeSpecimen();
                    driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition[Math.min(beforeTakeWallSpecimenPosition.length-1 , beforeTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(470 , 330 , 0.25);
                }
                ,
                new Node[]{takeWallSpecimen}
        );

        takeWallSpecimen.addConditions(
                ()->{
                    if(intake.sampleInDeposit)intake.setState(Intake.State.REVERSE_UP);
                    intake.setState(Intake.State.REPAUS_UP);
                    driveTrain.setTargetPosition(takeWallSpecimenPosition[Math.min(takeWallSpecimenPosition.length-1 , takeWallSpecimen.index)]);
                    if(driveTrain.inPosition(50 , 200 ,0.2) || Math.abs(Odo.xVelocity)<5)
                    outtake.grabSample();
                }
                ,
                ()->{
                    return outtake.claw.state==outtake.claw.states.get("closeSpecimen") || outtake.claw.state==outtake.claw.states.get("goCloseSpecimen");
                }
                ,
                new Node[]{afterTakeWallSpecimen , afterTakeWallSpecimen , afterTakeWallSpecimen , afterTakeWallSpecimen , afterTakeWallSpecimen , afterTakeWallSpecimen , takeWallSpecimen}
        );
        afterTakeWallSpecimen.addConditions(
                ()->{
                    if(intake.sampleInDeposit)intake.setState(Intake.State.REVERSE_UP);
                    driveTrain.setTargetPosition(afterTakeWallSpecimenPosition[Math.min(afterTakeWallSpecimenPosition.length-1 , afterTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(1000 , 1000 , 0.5);
                }
                ,
                new Node[]{beforePutSpecimen}
        );


        currentNode=beforePutSpecimen;
    }

    ElapsedTime timer=new ElapsedTime();
    public void run(Telemetry telemetry)
    {

        currentNode.run();

        Odo.update();
        driveTrain.update();

        intake.update();
        outtake.update();
        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }
}
