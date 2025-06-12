package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class SpecimenAuto {

    public static Pose2D[] beforePutSpecimenPosition = {
            new Pose2D(800 ,100 ,0 ) ,
            new Pose2D(800 ,-20 ,-0.3 ) ,
            new Pose2D ( 830 ,-140 ,-0.3)
    };
    public static Pose2D[] putSpecimenPosition = {
            new Pose2D ( 840 , 100 ,0) ,
            new Pose2D ( 840 , 100 ,0) ,
            new Pose2D (915 ,-80 ,-0.35) ,
            new Pose2D (915 ,-80 ,-0.35) ,
            new Pose2D (915 ,-80 ,-0.35) ,
            new Pose2D (915 ,-80 ,-0.35) ,
            new Pose2D (915 ,-80 ,-0.35) ,
    };

    public static Pose2D[] releaseSamplePosition = {
            new Pose2D(400 ,-500 ,2.1)
    };

    public static Pose2D[] takeFloorSamplePosition = {
            new Pose2D(500 ,-850 ,0.38),
            new Pose2D(500 ,-850 ,0.8),
            new Pose2D(500 ,-900,0.93)
    };

    public static Pose2D throwPosition = new Pose2D(500, -850 ,2.2);

    public static Pose2D beforeTakeWallSpecimenPosition[] ={
            new Pose2D(100 ,-700 ,0) ,
            new Pose2D ( 100 , -700 ,0) ,
            new Pose2D(100 ,-700 ,-0.55) ,
            new Pose2D( 100 ,-700 ,-0.55) ,
            new Pose2D(100 ,-700, -0.55)
    };

    public static Pose2D[] takeWallSpecimenPosition = {
            new Pose2D(0 ,-700 ,0) ,
            new Pose2D(0 ,-700 ,0) ,
            new Pose2D(20 ,-700 ,0),
            new Pose2D(20 ,-700 ,0),
            new Pose2D(20 ,-700 ,0),

    };

    public static Pose2D[] afterTakeWallSpecimenPosition={
            new Pose2D( 100 , -570 , -0.55)
    };

    public static double releaseSampleExtendoPosition=750;
    public static double[] extendoPositionPreextend={0, 250 , 250 , 0};
    public static double[] takeFromFloorEstendoPosition={490 ,610 ,840 , 0};
    public static double throwExtendoPosition = 400;
    boolean tookFromFLoor=false;

    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;


    ElapsedTime timerVerifying=new ElapsedTime();
    ElapsedTime timerToWait =new ElapsedTime();
    ElapsedTime timerTryingToTake = new ElapsedTime();
    ElapsedTime timerWaitingTryAgain = new ElapsedTime();
    ElapsedTime timerToReverse = new ElapsedTime();
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
        driveTrain = new MecanumDriveTrain(MecanumDriveTrain.State.PID);


        beforePutSpecimen.addConditions(
                ()->{
                    outtake.goUp();
                    driveTrain.setTargetPosition( beforePutSpecimenPosition[Math.min(beforePutSpecimenPosition.length-1 , beforePutSpecimen.index)]);
                    intake.setState(Intake.State.REPAUS_DOWN);
                }
                ,
                ()->{
                    return (driveTrain.inPosition(500 , 500 , 0.7) || (Odo.odo.getVelX()<30 && Odo.getX()>600)) && outtake.inPosition() && (outtake.lift.state== Lift.State.GOING_UP || outtake.lift.state== Lift.State.UP);
                }
                ,
                new Node[]{putSpecimen}
        );

        putSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(putSpecimenPosition[Math.min(putSpecimenPosition.length-1 , putSpecimen.index)]);
                    if(driveTrain.inPosition(40 , 100 , 0.6) || (driveTrain.inPosition(200 , 280 , 0.5) && putSpecimen.index>1))outtake.score();
                    intake.asure1inDepositState= Intake.Asure1inDeposit.Free;
                }
                ,
                ()->{
                    if(putSpecimen.index<=2 && driveTrain.inPosition(40 , 40 , 0.5))
                    Limelight.update();
                    timerTryingToTake.reset();
                    prepareToTakeFromSub.index=0;
                    takeFromSub.index=0;
                    takeAgain.index=0;
                    if((driveTrain.inPosition(200 , 280 , 0.5) && outtake.state!= Outtake.State.Up && putSpecimen.index>1) ||(driveTrain.inPosition(10 , 20 , 0.2)) || (Odo.odo.getVelX()<9 && Odo.getX()>600))
                    {
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
                    outtake.takeSpecimen();
                    if(MecanumDriveTrain.targetHeading==0)Limelight.update();

                    if(Limelight.extendoPosition!=-1)
                    {if(intake.extendo.state== Extendo.State.IN)
                    intake.setState(Intake.State.REPAUS_UP);}
                }
                ,
                ()->{
                    if(intake.ramp.state==intake.ramp.states.get("up") && MecanumDriveTrain.targetHeading==0 && Limelight.extendoPosition!=-1) {
                        driveTrain.setTargetPosition(Odo.getX(), Odo.getY(), -Odo.getHeading() + Limelight.targetAngle);
                        timerTryingToTake.reset();
                    }

                    if(intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN)intake.setState(Intake.State.INTAKE_DOWN);

                    if(timerTryingToTake.seconds()>1.7)
                    {
                        intake.setExtendoIN();
                        if(!tookFromFLoor)
                        prepareToTakeFromSub.next[0]=takeFloorSample;
                        else prepareToTakeFromSub.next[0]=beforeTakeWallSpecimen;
                        return true;
                    }
                    if(driveTrain.inPosition(1000 , 1000 , 0.08) && MecanumDriveTrain.targetHeading!=0)
                    intake.setExtendoTargetPosition(Limelight.extendoPosition-15);
                    if( intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN && intake.ramp.state==intake.ramp.states.get("down"))
                    {
                        timerTryingToTake.reset();
                        return true;
                    }
                    return false;
                }
                ,
                new Node[]{takeFromSub}
        );

        takeFromSub.addConditions(
                ()->{
                    outtake.takeSpecimen();
                    if(intake.asure1inDepositState== Intake.Asure1inDeposit.Free)
                    {
                        intake.setExtendoTargetPosition(Limelight.extendoPosition+130);
                    intake.setState(Intake.State.INTAKE_DOWN);}
                }
                ,
                ()->{
                    if(timerTryingToTake.seconds()>1.75)
                    {
                        takeFromSub.next[0]=takeAgain;
                        if(!tookFromFLoor)
                        takeFromSub.next[1]=takeFloorSample;
                        else takeFromSub.next[1]=beforeTakeWallSpecimen;
                        if(takeFromSub.index==1){intake.extendo.setIn(); intake.setState(Intake.State.REVERSE_UP);}
                        timerWaitingTryAgain.reset();
                        return true;
                    }
                    else{
                        takeFromSub.next[0]=prepareReleaseSampleFromSub;
                        takeFromSub.next[1]=prepareReleaseSampleFromSub;
                    }

                    if(intake.sampleInDeposit && timerVerifying.seconds()>0.15)
                        timerVerifying.reset();
                    if(intake.sampleInDeposit && this.state==intake.sampleColor.state && timerVerifying.seconds()>0.08 && timerVerifying.seconds()<0.15){
                        intake.asure1inDepositState= Intake.Asure1inDeposit.PrepareToClean;
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
                    driveTrain.setTargetPosition(Odo.getX() , Odo.getY() , 0);
                    timerTryingToTake.reset();
                    Limelight.extendoPosition=-1;
                    MecanumDriveTrain.targetHeading=0;
                }
                ,
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)intake.setState(Intake.State.REPAUS_DOWN);
                    return driveTrain.inPosition() && intake.ramp.state==intake.ramp.states.get("down") && intake.state== Intake.State.REPAUS_DOWN && timerToWait.seconds()>0.15;
                }
                ,
                new Node[]{prepareToTakeFromSub}
        );

        prepareReleaseSampleFromSub.addConditions(
                ()->{
                    if(intake.extendo.state== Extendo.State.GOING_IN && MecanumDriveTrain.targetHeading==1.95 && driveTrain.inPosition(1000 , 1000 , 0.5))
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , prepareReleaseSampleFromSub.index)]);
                    else if(intake.extendo.encoder.getPosition()<200)driveTrain.setTargetPosition(Odo.getX() , Odo.getY() , 1.95);
                }
                ,
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)intake.asure1inDepositState= Intake.Asure1inDeposit.Free;

                    return driveTrain.inPosition(300 , 300 , 1) && MecanumDriveTrain.targetHeading==1.95;
                }
                ,
                new Node[]{releaseSample}
        );

        releaseSample.addConditions(
                ()->{
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , releaseSample.index)]);
                    if(driveTrain.inPosition(1000 , 1000 , 0.55))
                        if(intake.extendo.state== Extendo.State.IN)intake.asure1inDepositState= Intake.Asure1inDeposit.Free;

                    intake.setExtendoTargetPosition(releaseSampleExtendoPosition);

                    if(driveTrain.inPosition(1000 , 500 , 0.5))intake.setState(Intake.State.REVERSE_UP);
                }
                ,
                ()->{
                    if(tookFromFLoor)releaseSample.next[0]=beforeTakeWallSpecimen;
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.5)timerToReverse.reset();
                    if(timerToReverse.seconds()>0.15 && timerToReverse.seconds()<0.5){ timerTryingToTake.reset(); intake.setExtendoIN(); return true;}
                    return false;
                }
                ,
                new Node[]{takeFloorSample , beforeTakeWallSpecimen }

        );

        takeFloorSample.addConditions(
                ()->{
                    tookFromFLoor=true;
                    driveTrain.setTargetPosition(takeFloorSamplePosition[Math.min(takeFloorSamplePosition.length-1 , takeFloorSample.index)]);

                    if(intake.extendo.encoder.getPosition()>50 && intake.extendo.state!= Extendo.State.GOING_IN)intake.setState(Intake.State.INTAKE_DOWN);
                    else if(takeFloorSample.index==0)intake.setState(Intake.State.REVERSE_UP);

                    if(!driveTrain.inPosition(105 , 105 , 0.43) && intake.extendo.state!= Extendo.State.GOING_IN){timerTryingToTake.reset();}
                    else if(driveTrain.inPosition(105 , 105 , 0.43)){
                        intake.setExtendoTargetPosition(takeFromFloorEstendoPosition[takeFloorSample.index]);
                        if(intake.extendo.encoder.getPosition()>50 && intake.extendo.state!= Extendo.State.GOING_IN)
                        intake.setState(Intake.State.INTAKE_DOWN);
                    }

                }
                ,
                ()->{
                    return (intake.sampleInDeposit || timerTryingToTake.seconds()>1) && driveTrain.inPosition(105 , 105 , 0.42);
                }
                ,
                new Node[]{throwNode}
        );

        throwNode.addConditions(
                ()->{
                    driveTrain.setTargetPosition(throwPosition);
                    intake.setExtendoTargetPosition(throwExtendoPosition);

                    if(driveTrain.inPosition(1000 , 1000 , 1))intake.setState(Intake.State.REVERSE_UP);
                }
                ,
                ()->{
                    timerTryingToTake.reset();
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.3 && driveTrain.inPosition(1000 , 1000 ,1))timerToReverse.reset();
                    if(timerToReverse.seconds()>0.1 && timerToReverse.seconds()<0.3 && driveTrain.inPosition(1000 , 1000 , 1))
                    {intake.setExtendoTargetPosition(extendoPositionPreextend[releaseSample.index+1]);return true;}
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
                    return( (driveTrain.inPosition(380 , 380 , 0.65)&& beforeTakeWallSpecimen.index>0) || (driveTrain.inPosition(45 ,45 , 0.15) && beforeTakeWallSpecimen.index==0));
                }
                ,
                new Node[]{takeWallSpecimen}
        );

        takeWallSpecimen.addConditions(
                ()->{
                    intake.setState(Intake.State.REPAUS_UP);
                    driveTrain.setTargetPosition(takeWallSpecimenPosition[Math.min(takeWallSpecimenPosition.length-1 , takeWallSpecimen.index)]);
                    if(driveTrain.inPosition(35 , 100 ,0.2) )
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
                    driveTrain.setTargetPosition(afterTakeWallSpecimenPosition[Math.min(afterTakeWallSpecimenPosition.length-1 , afterTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(1000 , 1000 , 0.37);
                }
                ,
                new Node[]{beforePutSpecimen}
        );


        currentNode=beforePutSpecimen;
    }

    public void run(Telemetry telemetry)
    {
        currentNode.run();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];



        Odo.update();

        driveTrain.update();
        intake.update();
        outtake.update();
    }
}
