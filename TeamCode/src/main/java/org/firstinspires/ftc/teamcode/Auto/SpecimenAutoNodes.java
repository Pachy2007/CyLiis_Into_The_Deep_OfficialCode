package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Modules.Others.SampleColor.State.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.ExtendoBlocker;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Others.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Color;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

import javax.tools.JavaCompiler;

@Config
public class SpecimenAutoNodes {

    public static double[] intakeExtendoPosition={450 ,785 ,1170};
    public static double[] reverseExtendoPosition={650 ,650 ,650};

    public static Pose2D beforePutSpecimenPosition=new Pose2D (-700 , 100 ,-0.2);
    public static Pose2D putSpecimenPosition=new Pose2D(-815 ,30 ,-0.28);
    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{
            new Pose2D(-500 ,850 ,0.3),
            new Pose2D(-500 ,850 ,0.77),
            new Pose2D(-500 ,850,0.967)
    };
    public static Pose2D[] releaseSamplePosition=new Pose2D[]{
            new Pose2D(-500, 850 ,2.1),
            new Pose2D(-500, 850 ,2.1),
            new Pose2D(-500, 850 ,2.1)
    };
    public static Pose2D beforeTakeWallSpecimenPosition=new Pose2D(-200 ,670 ,0.1);
    public static Pose2D takeWallSpecimenPosition=new Pose2D(-10 ,680 ,0);
    public static Pose2D try6Position=new Pose2D(-820 , -200 , 0);
    public boolean skip=false;
    boolean p=false;

    public static double xTry=-820 , yTry=-250 , headingTry=0;
    static double angle;


    public ElapsedTime timerOuttake;
    Node beforePutSpecimen , putSpecimen , takeFloorSample , releaseSample , beforeTakeWallSpecimen ,takeWallSpecimen , goTry6 , try6, releaseSampleFromSub, detectSample,takeSample, goToHuman, prepareToTryAgain,prepareTakeFloorSample, prepareGoToHuman;

    boolean another=false;
    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Extendo extendo;
    boolean second=false;
    public Latch latch;
    boolean sixth=false;
    public ExtendoBlocker extendoBlocker;

    SampleColor sample;
    DigitalChannel bb;

    public boolean INIT=false;

    ElapsedTime timer2ForSub;
    ElapsedTime timerForSub;
    boolean a=false;

    ElapsedTime timer;
    ElapsedTime reverseTimer;

    Node currentNode;
    public void run(HardwareMap hardwareMap , Telemetry telemetry)
    {
        if(!INIT)
        {
            beforePutSpecimenPosition=new Pose2D (-655 , 170 ,-0.55);
            putSpecimenPosition=new Pose2D(-835 ,55 ,-0.45);

            Limelight.init(hardwareMap);

            beforeTakeWallSpecimen=new Node("beforeTakeWallSpecimen");
            beforePutSpecimen=new Node("beforePutSpecimen");
            putSpecimen=new Node("putSpecimen");
            takeFloorSample=new Node("takeFloorSample");
            releaseSample=new Node("releaseSample");
            takeWallSpecimen=new Node("takeWallSpecimen");
            goTry6=new Node("goTry6");
            try6=new Node("try6");
            releaseSampleFromSub=new Node("releaseSampleFromSub");
            detectSample=new Node("detectSample");
            takeSample=new Node("takeSample");
            prepareToTryAgain=new Node("prepareToTryAgain");
            goToHuman=new Node("goToHuman");
            prepareGoToHuman=new Node("prepareGoToHuman");
            prepareTakeFloorSample=new Node("prepareTakeFloorSample");

            extendoBlocker=new ExtendoBlocker();
            timerForSub=new ElapsedTime();

            Hardware.init(hardwareMap);
          //  Limelight.init(hardwareMap);
            INIT=true;

            sample=new SampleColor();

            timer2ForSub=new ElapsedTime();
            timer2ForSub.startTime();
            timerForSub.startTime();
            bb=hardwareMap.get(DigitalChannel.class , "bb");
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            outtake.haveSample=false;
            extendo=new Extendo();
            latch=new Latch();

            timerOuttake=new ElapsedTime();
            reverseTimer=new ElapsedTime();
            timer=new ElapsedTime();




            beforePutSpecimen.addConditions(
                    ()->{



                        if(outtake.arm.state==outtake.arm.states.get("withElementSpecimen"))
                        {
                            if(beforePutSpecimen.index==0){beforePutSpecimenPosition.x=-600;driveTrain.setTargetPosition(beforePutSpecimenPosition);}
                            else {beforePutSpecimenPosition.x=-680;
                            }
                            outtake.goUp();}
                        driveTrain.setTargetPosition(beforePutSpecimenPosition);
                    }
                    ,
                    ()->{
                        return (((driveTrain.inPosition(120 , 50 , 0.25 ) && beforePutSpecimen.index!=0) || (beforePutSpecimen.index==0 && driveTrain.inPosition(75 ,15 , 0.1) && timerOuttake.seconds()>1)) && outtake.inPosition() && driveTrain.targetX<-500);
                    }
                    ,
                    new Node[]{putSpecimen}
            );

            putSpecimen.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(putSpecimenPosition);
                    }
                    ,
                    ()->{
                        timerForSub.reset();
                        timer2ForSub.reset();
                        timer.reset();
                        if(driveTrain.inPosition(30 , 20 , 0.45) || (Math.abs(Odo.odo.getVelX())<2 && driveTrain.x<-740)){
                            outtake.score();return true;}
                        return false;

                    }
                    ,
                    new Node[]{goTry6 , beforeTakeWallSpecimen}
            );
            goTry6.addConditions(
            ()->{
                outtake.goDefault();
                driveTrain.setTargetPosition(try6Position);
                intake.setState(Intake.State.REPAUS_DOWN);
                timerForSub.reset();
                timer2ForSub.reset();
            }
                    ,
                    ()->{

                        return driveTrain.inPosition();
                    }
                    ,
                    new Node[]{detectSample}

            );
            detectSample.addConditions(
                    ()->{
                        if(timerForSub.seconds()<0.1)
                        {Limelight.update();
                        angle=-Odo.getHeading()+Math.atan(Limelight.X/Limelight.Y);}
                        else{
                            if(MecanumDriveTrain.targetHeading==0)
                            driveTrain.setTargetPosition(-820 , -200 , angle);
                            intake.setState(Intake.State.REPAUS_UP);
                        }


                    }
                    ,
                    ()->{
                        return driveTrain.inPosition() && intake.inPosition() && MecanumDriveTrain.targetHeading!=0;
                    }
                    ,new Node[]{takeSample}
            );
            takeSample.addConditions(
                    ()->{

                        driveTrain.setTargetPosition(-820 , -200 , angle);
                        extendo.setTargetPosition(Limelight.extendoPosition);
                        if(extendo.inPosition()){
                            intake.setState(Intake.State.INTAKE_DOWN);
                        }
                        else timerForSub.reset();
                    }
                    ,
                    ()->{
                        if(timerForSub.seconds()>2)
                        {
                            takeSample.next[0]=prepareToTryAgain;
                            takeSample.next[1]=prepareTakeFloorSample;
                        }
                        else {takeSample.next[0]=prepareGoToHuman;takeSample.next[1]=prepareGoToHuman;}

                        timer2ForSub.reset();
                        return timerForSub.seconds()>2 || (!bb.getState() && sample.state== RED);
                    },
                   new Node[]{prepareToTryAgain , prepareGoToHuman}
            );
            prepareToTryAgain.addConditions(
                    ()->{
                        if(extendo.inPosition() && intake.inPosition())
                        extendo.setIn();
                        if(extendo.state== Extendo.State.IN && intake.state!= Intake.State.REVERSE_UP)
                        {
                            extendo.setTargetPosition(300);
                            intake.setState(Intake.State.REVERSE_UP);
                        }
                    }
                    ,
                    ()->{
                        return driveTrain.inPosition() && extendo.state== Extendo.State.IN && intake.state== Intake.State.REVERSE_UP;
                    }
                    ,
                    new Node[]{takeSample}
            );
            prepareGoToHuman.addConditions(
                    ()->{
                        sixth=true;
                        if(timer2ForSub.seconds()<=0.35)
                        latch.setState("goOpen");

                        if(latch.state==latch.states.get("open") && timer2ForSub.seconds()<=0.35)intake.setState(Intake.State.REVERSE_UP);
                        else if(latch.state==latch.states.get("open") && timer2ForSub.seconds()>0.35){latch.setState("goClose");intake.setState(Intake.State.REPAUS_UP);}
                        else if(latch.state==latch.states.get("close")){extendo.setIn();intake.setState(Intake.State.INTAKE_UP);}
                        else if(latch.state!=latch.states.get("goClose"))timerForSub.reset();}
                    ,
                    ()->{
                        if(extendo.state== Extendo.State.IN){return true;}
                            return false;
                    }
                    ,
                    new Node[]{goToHuman}
            );
            prepareTakeFloorSample.addConditions(
                    ()->{
                        intake.setState(Intake.State.REVERSE_UP);
                        intake.update();
                        if(intake.ramp.inPosition())extendo.setIn();
                    }
                    ,
                    ()->{
                        return intake.ramp.inPosition();
                    }
                    ,
                    new Node[]{takeFloorSample}
            );
            goToHuman.addConditions(
                    ()->{
                        sixth=true;
                        driveTrain.setTargetPosition(-200 , 690 , -0.4);
                        if(extendo.state== Extendo.State.IN){
                        if(!bb.getState() && outtake.state== Outtake.State.Deafult && p==false)
                        outtake.grabSample();p=true;
                        if(outtake.state== Outtake.State.DeafultWithElement)outtake.releaseSample();
                        outtake.update();
                        if(driveTrain.inPosition() && outtake.state== Outtake.State.ReleaseSample && outtake.arm.inPosition() && reverseTimer.seconds()>2)reverseTimer.reset();
                            if(reverseTimer.seconds()>0.05 &&reverseTimer.seconds()<1)
                            outtake.releaseSample();}
                    }
                    ,
                    ()->{
                        timer.reset();
                        return bb.getState() && outtake.claw.state!=outtake.claw.states.get("close") && outtake.claw.state!=outtake.claw.states.get("goClose") && driveTrain.inPosition();
                    }
                    ,
                    new Node[]{takeFloorSample}
            );


            takeFloorSample.addConditions(
                    ()->{
                        if(takeSample.index==0 && !driveTrain.inPosition())extendo.setIn();
                        latch.setState("goClose");
                        outtake.goDefault();
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takeFloorSample.index]);
                        if(driveTrain.inPosition(20 , 20 , 0.28) && Math.abs(Odo.odo.getHeadingVelocity())<4)
                        {extendo.setTargetPosition(intakeExtendoPosition[takeFloorSample.index]);
                        intake.setState(Intake.State.INTAKE_DOWN);}
                        else timer.reset();

                    }
                    ,
                    ()->{
                        if(timer.seconds()>2){return true;}
                        return !bb.getState();
                    }
                    ,
                    new Node[]{releaseSample}

            );
            releaseSample.addConditions(
                    ()->{
                        timer.reset();
                        driveTrain.setTargetPosition(releaseSamplePosition[releaseSample.index]);
                        if(driveTrain.inPosition(30 , 30 , 0.4) || releaseSample.index>0)
                        extendo.setTargetPosition(reverseExtendoPosition[releaseSample.index]);

                        if(driveTrain.inPosition(30 , 30 , 0.4))intake.setState(Intake.State.REVERSE_DOWN);
                    }
                    ,
                    ()->{
                        if(reverseTimer.seconds()>1 && driveTrain.inPosition(30 , 30 , 0.2))reverseTimer.reset();
                        if(reverseTimer.seconds()>0.3 && reverseTimer.seconds()<1 && bb.getState()){extendo.setTargetPosition(300);return true;}
                        return false;
                    }
                    ,
                    new Node[]{takeFloorSample , takeFloorSample , beforeTakeWallSpecimen}
            );
            beforeTakeWallSpecimen.addConditions(
                    ()->{
                        extendo.setIn();
                        if(extendo.state== Extendo.State.IN)extendoBlocker.setState("goClose");
                        driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition);

                        outtake.takeSpecimen();
                        intake.setState(Intake.State.REPAUS_UP);
                    }
                    ,
                    ()->{

                        return (outtake.inPosition() && driveTrain.inPosition(100 ,40 , 0.25) && outtake.state== Outtake.State.Specimen && outtake.arm.state==outtake.arm.states.get("takeSpecimen"));
                    }
                    ,
                    new Node[]{takeWallSpecimen}
            );

            takeWallSpecimen.addConditions(
                    ()->{

                        if(takeWallSpecimen.index>=5 || (!sixth && takeWallSpecimen.index>=4))
                        {
                            if(!a)
                            driveTrain.setTargetPosition(takeWallSpecimenPosition);
                            if(driveTrain.inPosition(20 , 20 , 0.15))
                            {
                                outtake.goDefault();
                                intake.setState(Intake.State.REPAUS_UP);
                            }


                        }
                        else{
                        driveTrain.setTargetPosition(takeWallSpecimenPosition);
                        if(driveTrain.inPosition(25 ,100 , 0.3))outtake.grabSample();}
                    }
                    ,
                    ()->{
                        if(sixth)takeWallSpecimen.next[4]=beforePutSpecimen;
                        if(outtake.claw.state==outtake.claw.states.get("closeSpecimen") && !a){timerOuttake.reset();return true;}
                        return false;
                    }
                    ,
                    new Node[]{beforePutSpecimen , beforePutSpecimen , beforePutSpecimen , beforePutSpecimen , takeWallSpecimen, takeWallSpecimen}
            );

            currentNode=beforePutSpecimen;
        }
        else {
        currentNode.run();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];



        Odo.update();

        driveTrain.update();
        intake.update();
        extendo.update();
        outtake.update();
        latch.update();
        sample.update();}

    }




}
