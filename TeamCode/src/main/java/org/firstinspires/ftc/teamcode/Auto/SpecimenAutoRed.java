package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class SpecimenAutoRed {

    public static Pose2D[] beforePutSpecimenPosition = {
            new Pose2D(800 ,100 ,0 ) ,
            new Pose2D ( 820 ,-150 ,-0.4)
    };
    public static Pose2D[] putSpecimenPosition = {
            new Pose2D ( 800 , 100 ,0) ,
            new Pose2D (850 ,0 ,-0.4)
    };

    public static Pose2D[] releaseSamplePosition = {
            new Pose2D(250 ,-300 ,1.95)
    };

    public static Pose2D[] takeFloorSamplePosition = {
            new Pose2D(500 ,-850 ,0.37),
            new Pose2D(500 ,-850 ,0.75),
            new Pose2D(500 ,-900,0.91)
    };

    public static Pose2D throwPosition = new Pose2D(500, -850 ,2.2);

    public static Pose2D beforeTakeWallSpecimenPosition[] ={
            new Pose2D(100 ,-685 ,0) ,
            new Pose2D(50 ,-685 ,-0.3) ,
            new Pose2D(50 ,-685 ,0.3) ,
            new Pose2D(50 ,-685, -0.3)
    };

    public static Pose2D[] takeWallSpecimenPosition = {
            new Pose2D(0 ,-670 ,0)
    };

    public static Pose2D[] afterTakeWallSpecimenPosition={
            new Pose2D( 50 , -600 , -0.4)
    };

    public static double releaseSampleExtendoPosition=600;
    public static double[] extendoPositionPreextend={100, 250 , 300 , 0};
    public static double[] takeFromFloorEstendoPosition={350 ,510 ,750 , 0};
    public static double throwExtendoPosition = 300;

    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;

    ElapsedTime timerTryingToTake = new ElapsedTime();
    ElapsedTime timerWaitingTryAgain = new ElapsedTime();
    ElapsedTime timerToReverse = new ElapsedTime();
    Node beforePutSpecimen , putSpecimen , prepareToTakeFromSub , takeFromSub , takeAgain , prepareReleaseSampleFromSub,releaseSampleFromSub , takeFloorSample , releaseSample , throwNode , beforeTakeWallSpecimen , takeWallSpecimen , afterTakeWallSpecimen;
    Node currentNode;

    public void init(HardwareMap hardwareMap)
    {

        Hardware.init(hardwareMap);
        Limelight.init(hardwareMap , 0);

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
                    if(beforePutSpecimen.index==0)intake.setState(Intake.State.REPAUS_DOWN);
                }
                ,
                ()->{
                    return driveTrain.inPosition(300 , 300 , 0.6) ;
                }
                ,
                new Node[]{putSpecimen}
        );

        putSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(putSpecimenPosition[Math.min(putSpecimenPosition.length-1 , putSpecimen.index)]);
                    if(driveTrain.inPosition(30 , 70 , 0.4))outtake.score();
                }
                ,
                ()->{
                    if(putSpecimen.index==0)
                    {
                        driveTrain.inPosition(20 , 150 , 0.6);
                        Limelight.update();
                    }
                    return driveTrain.inPosition(20 , 150 , 0.6) && outtake.state!= Outtake.State.Up;
                }
                ,
                new Node[]{prepareToTakeFromSub , beforeTakeWallSpecimen}

        );

        prepareToTakeFromSub.addConditions(
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)
                    intake.setState(Intake.State.REPAUS_UP);
                }
                ,
                ()->{
                    if(intake.ramp.state==intake.ramp.states.get("up") && MecanumDriveTrain.targetHeading==0) {
                        driveTrain.setTargetPosition(Odo.getX(), Odo.getY(), -Odo.getHeading() + Math.atan(Limelight.X / Limelight.Y));
                        timerTryingToTake.reset();
                    }
                    if(intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN)intake.setState(Intake.State.INTAKE_DOWN);

                    if(driveTrain.inPosition(1000 , 1000 , 0.25) && MecanumDriveTrain.targetHeading!=0)
                    intake.setExtendoTargetPosition(Limelight.extendoPosition);


                    return intake.extendo.inPosition() && intake.extendo.state!= Extendo.State.IN && intake.ramp.state==intake.ramp.states.get("down");
                }
                ,
                new Node[]{takeFromSub}
        );

        takeFromSub.addConditions(
                ()->{
                    if(intake.asure1inDepositState== Intake.Asure1inDeposit.Free)
                    {
                        intake.setExtendoTargetPosition(Limelight.extendoPosition+200);
                    intake.setState(Intake.State.INTAKE_DOWN);}
                }
                ,
                ()->{
                    if(timerTryingToTake.seconds()>3)
                    {
                        takeFromSub.next[0]=takeAgain;
                        takeFromSub.next[1]=takeFloorSample;
                        if(takeFromSub.index==1){intake.extendo.setIn(); intake.setState(Intake.State.REPAUS_UP);}
                        timerWaitingTryAgain.reset();
                        return true;
                    }

                    if(intake.sampleInDeposit){
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
                    intake.setState(Intake.State.REPAUS_UP);
                    timerTryingToTake.reset();
                }
                ,
                ()->{
                    return timerWaitingTryAgain.seconds()>1;
                }
                ,
                new Node[]{takeFromSub}
        );

        prepareReleaseSampleFromSub.addConditions(
                ()->{
                    if(intake.extendo.state== Extendo.State.GOING_IN && MecanumDriveTrain.targetHeading==1.95 && driveTrain.inPosition(1000 , 1000 , 0.5))
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , prepareReleaseSampleFromSub.index)]);
                    else if(intake.extendo.encoder.getPosition()<300)driveTrain.setTargetPosition(Odo.getX() , Odo.getY() , 1.95);
                }
                ,
                ()->{
                    if(intake.extendo.state== Extendo.State.IN)intake.asure1inDepositState= Intake.Asure1inDeposit.Free;
                    return intake.extendo.state== Extendo.State.IN;
                }
                ,
                new Node[]{releaseSample}
        );

        releaseSample.addConditions(
                ()->{
                    driveTrain.setTargetPosition(releaseSamplePosition[Math.min(releaseSamplePosition.length-1 , releaseSample.index)]);
                    if(driveTrain.inPosition(1000 , 1000 , 0.5))
                    intake.setExtendoTargetPosition(releaseSampleExtendoPosition);

                    if(driveTrain.inPosition(1000 , 300 , 0.5))intake.setState(Intake.State.REVERSE_DOWN);
                }
                ,
                ()->{
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.5)timerToReverse.reset();
                    if(timerToReverse.seconds()>0.07 && timerToReverse.seconds()<0.5)return true;
                    return false;
                }
                ,
                new Node[]{takeFloorSample}

        );

        takeFloorSample.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeFloorSamplePosition[Math.min(takeFloorSamplePosition.length-1 , takeFloorSample.index)]);
                    if(intake.extendo.encoder.getPosition()>50)
                        if(intake.extendo.inPosition())intake.setState(Intake.State.INTAKE_DOWN);

                    if(!driveTrain.inPosition(70 , 70 , 0.2)){if(takeFloorSample.index==0)intake.setExtendoTargetPosition(100); timerTryingToTake.reset();}
                    else{
                        intake.setExtendoTargetPosition(takeFromFloorEstendoPosition[takeFloorSample.index]);
                        if(intake.extendo.encoder.getPosition()>50)
                        if(intake.extendo.inPosition())intake.setState(Intake.State.INTAKE_DOWN);
                    }
                }
                ,
                ()->{
                    return (intake.sampleInDeposit || timerTryingToTake.seconds()>1) && driveTrain.inPosition();
                }
                ,
                new Node[]{throwNode}
        );

        throwNode.addConditions(
                ()->{
                    driveTrain.setTargetPosition(throwPosition);
                    intake.setExtendoTargetPosition(throwExtendoPosition);

                    if(driveTrain.inPosition(1000 , 1000 , 0.9))intake.setState(Intake.State.REVERSE_DOWN);
                }
                ,
                ()->{
                    timerTryingToTake.reset();
                    if(!intake.sampleInDeposit && timerToReverse.seconds()>0.3 && driveTrain.inPosition(1000 , 1000 ,0.9))timerToReverse.reset();
                    if(timerToReverse.seconds()>0.07 && timerToReverse.seconds()<0.3 && driveTrain.inPosition(1000 , 1000 , 0.9))
                    {intake.setExtendoTargetPosition(extendoPositionPreextend[releaseSample.index+1]);return true;}
                    return false;
                }
                ,
                new Node[]{takeFloorSample , takeFloorSample , beforeTakeWallSpecimen}
        );

        beforeTakeWallSpecimen.addConditions(
                ()->{
                    intake.setState(Intake.State.REPAUS_UP);
                    intake.setExtendoIN();
                    outtake.takeSpecimen();
                    driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition[Math.min(beforeTakeWallSpecimenPosition.length-1 , beforeTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return outtake.arm.inPosition() && ( (driveTrain.inPosition(400 , 70 , 0.2)&& beforeTakeWallSpecimen.index>0) || (driveTrain.inPosition(45 ,45 , 0.15) && beforeTakeWallSpecimen.index==0));
                }
                ,
                new Node[]{takeWallSpecimen}
        );

        takeWallSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeWallSpecimenPosition[Math.min(takeWallSpecimenPosition.length-1 , takeWallSpecimen.index)]);
                    if(driveTrain.inPosition(40 , 150 ,0.5))
                    outtake.grabSample();
                }
                ,
                ()->{
                    return outtake.claw.state==outtake.claw.states.get("closeSpecimen") || outtake.claw.state==outtake.claw.states.get("goCloseSpecimen");
                }
                ,
                new Node[]{afterTakeWallSpecimen}
        );
        afterTakeWallSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(afterTakeWallSpecimenPosition[Math.min(afterTakeWallSpecimenPosition.length-1 , afterTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(1000 , 1000 , 0.25);
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
