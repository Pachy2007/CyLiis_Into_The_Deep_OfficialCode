package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor.State.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class Specimen7 {

    public static double[] intakeExtendoPosition={450 ,800 ,1050};
    public static double[] reverseExtendoPosition={650 ,650 ,650};

    public static Pose2D beforePutSpecimenPosition=new Pose2D (-700 , 100 ,-0.2);
    public static Pose2D putSpecimenPosition=new Pose2D(-815 ,30 ,-0.28);
    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{
            new Pose2D(-500 ,850 ,0.41),
            new Pose2D(-500 ,850 ,0.8),
            new Pose2D(-500 ,850,0.975)
    };
    public static Pose2D[] releaseSamplePosition=new Pose2D[]{
            new Pose2D(-500, 850 ,2.1),
            new Pose2D(-500, 850 ,2.1),
            new Pose2D(-500, 850 ,2.1)
    };
    public static Pose2D beforeTakeWallSpecimenPosition=new Pose2D(-200 ,670 ,0.1);
    public static Pose2D takeWallSpecimenPosition=new Pose2D(-20 ,680 ,0);
    public static Pose2D try6Position=new Pose2D(-820 , -200 , 0);
    public static Pose2D takeFirstFromWal=new Pose2D(-80 ,1450 ,-Math.PI/2);

    static double angle;


    public ElapsedTime timerOuttake;
    Node beforePutSpecimen , putSpecimen , takeFloorSample , releaseSample , beforeTakeWallSpecimen ,takeWallSpecimen , goTry6 , try6, releaseSampleFromSub, detectSample,takeSample, goToHuman, prepareToTryAgain,prepareTakeFloorSample, prepareGoToHuman, takeFirstSample;

    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    boolean sixth=false;

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
            beforePutSpecimenPosition=new Pose2D (-600 , 200 ,-0.5);
            putSpecimenPosition=new Pose2D(-920 ,0 ,-0.5);

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
            takeFirstSample=new Node("takeFirstSample");

            timerForSub=new ElapsedTime();

            Hardware.init(hardwareMap);
            Limelight.init(hardwareMap);
            INIT=true;

            timer2ForSub=new ElapsedTime();
            timer2ForSub.startTime();
            timerForSub.startTime();
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake(RED , false);
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            outtake.haveSample=false;

            timerOuttake=new ElapsedTime();
            reverseTimer=new ElapsedTime();
            timer=new ElapsedTime();




            beforePutSpecimen.addConditions(
                    ()->{


                        if(beforePutSpecimen.index==0)intake.setState(Intake.State.REPAUS_DOWN);

                        if(outtake.arm.state==outtake.arm.states.get("withElementSpecimen"))
                        {
                            if(beforePutSpecimen.index==0){beforePutSpecimenPosition.x=-600;driveTrain.setTargetPosition(beforePutSpecimenPosition);}
                            else {beforePutSpecimenPosition.x=-680;
                            }
                            outtake.goUp();}
                        if(beforePutSpecimen.index!=0)
                        driveTrain.setTargetPosition(beforePutSpecimenPosition);
                        else driveTrain.setTargetPosition(-600, -200 , 0);
                        outtake.update();
                    }
                    ,
                    ()->{
                        return outtake.inPosition() && outtake.state== Outtake.State.Up;
                    }
                    ,
                    new Node[]{putSpecimen}
            );

            putSpecimen.addConditions(
                    ()->{
                        if(putSpecimen.index!=0)
                        driveTrain.setTargetPosition(putSpecimenPosition);
                        else driveTrain.setTargetPosition(-790, -200 , 0);
                    }
                    ,
                    ()->{
                        timerForSub.reset();
                        timer2ForSub.reset();
                        timer.reset();
                        if(driveTrain.inPosition(90 , 80 , 0.45) || (Math.abs(Odo.odo.getVelX())<2 && driveTrain.x<-740)){
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
                        timerForSub.reset();
                        if(driveTrain.inPosition())
                        {
                            Limelight.update();
                            angle=-Odo.getHeading()+Math.atan(Limelight.X/Limelight.Y);
                            return true;
                        }
                        return false;
                    }
                    ,
                    new Node[]{detectSample}

            );
            detectSample.addConditions(
                    ()->{
                            if(MecanumDriveTrain.targetHeading==0)
                            driveTrain.setTargetPosition(-820 , -200 , angle);
                            intake.setState(Intake.State.REPAUS_UP);
                    }
                    ,
                    ()->{
                        timer2ForSub.reset();
                        if(driveTrain.inPosition() && intake.inPosition() && MecanumDriveTrain.targetHeading!=0){timerForSub.reset();timerForSub.reset();}
                        return driveTrain.inPosition() && intake.inPosition() && MecanumDriveTrain.targetHeading!=0;
                    }
                    ,new Node[]{takeSample}
            );
            takeSample.addConditions(
                    ()->{

                        driveTrain.setTargetPosition(-820 , -200 , angle);
                        intake.setExtendoTargetPosition(Limelight.extendoPosition);
                        if(intake.extendo.inPosition()){
                            intake.setState(Intake.State.INTAKE_DOWN);
                        }
                        else {timerForSub.reset();}
                    }
                    ,
                    ()->{
                        if(timerForSub.seconds()>0.9)
                        {
                            takeSample.next[0]=prepareToTryAgain;
                            takeSample.next[1]=prepareToTryAgain;
                            takeSample.next[2]=prepareTakeFloorSample;
                        }
                        else {takeSample.next[0]=prepareGoToHuman;takeSample.next[1]=prepareGoToHuman;takeSample.next[2]=prepareGoToHuman;}
                        /**if(!intake.hasSample.getState() && timer2ForSub.seconds()>0.4)timer2ForSub.reset();
                        if(!intake.hasSample.getState())timerForSub.reset();

                            if(timerForSub.seconds()>1 || (!intake.hasSample.getState() && intake.sampleColor.state== RED && timer2ForSub.seconds()<0.4 && timer2ForSub.seconds()>0.15))
                            {                        timer2ForSub.reset();
                                return true;
                            }**/
                        if(timer2ForSub.seconds()>0.25 && timer2ForSub.seconds()<0.4)intake.setState(Intake.State.REVERSE_UP);
                        return false;
                    },
                   new Node[]{prepareToTryAgain , prepareToTryAgain , prepareGoToHuman}
            );
            prepareToTryAgain.addConditions(
                    ()->{
                        intake.setState(Intake.State.REPAUS_UP);
                        intake.update();
                        timerForSub.reset();
                    }
                    ,
                    ()->{
                        return driveTrain.inPosition() && intake.state== Intake.State.REPAUS_UP && intake.ramp.inPosition() && timer2ForSub.seconds()>0.4;
                    }
                    ,
                    new Node[]{takeSample}
            );
            prepareGoToHuman.addConditions(
                    ()-> {
                        sixth = true;
                        intake.latch.setState("goClose");

                        if (intake.latch.state == intake.latch.states.get("close") && timer2ForSub.seconds() > 0.1) {
                            intake.setState(Intake.State.REVERSE_UP);
                            intake.setExtendoIN();
                        }
                    }
                    ,
                    ()->{
                        if(intake.extendo.state== Extendo.State.IN || intake.extendo.state== Extendo.State.GOING_IN){intake.setState(Intake.State.REVERSE_UP);timer2ForSub.reset();return true;}
                            return false;
                    }
                    ,
                    new Node[]{goToHuman}
            );
            takeFirstSample.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(takeFirstFromWal);
                        outtake.takeSpecimen();
                        if(driveTrain.inPosition(200 , 200 , 1) && timer2ForSub.seconds()<0.1)intake.setState(Intake.State.REVERSE_DOWN);
                        else if(!driveTrain.inPosition(200 , 200 , 1)){intake.setState(Intake.State.INTAKE_UP);intake.update();intake.latch.setState("goOpen");timer2ForSub.reset();}
                    }
                    ,
                    ()->{
                        if(outtake.inPosition() && driveTrain.inPosition())outtake.grabSample();
                        return true;
                        //return driveTrain.inPosition() && intake.hasSample.getState() && (outtake.claw.state==outtake.claw.states.get("closeSpecimen") || outtake.claw.state==outtake.claw.states.get("goCloseSpecimen"));
                    }
                    ,
                    new Node[]{beforePutSpecimen}
            );

            prepareTakeFloorSample.addConditions(
                    ()->{
                        intake.setState(Intake.State.REVERSE_UP);
                        intake.update();
                        if(intake.ramp.inPosition())intake.setExtendoIN();
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
                        driveTrain.setTargetPosition(Odo.getX() , Odo.getY() , 2.1);

                        if(driveTrain.inPosition(1000 , 1000 , 1))
                        driveTrain.setTargetPosition(-300 , 400 , 2.1);

                        if(intake.extendo.state== Extendo.State.IN && driveTrain.inPosition(800 , 600 , 1) && driveTrain.targetX==-300)
                        {
                            intake.setExtendoTargetPosition(900);
                            intake.extendo.update();
                        }
                        if(Extendo.targetPosition!=900 && timer2ForSub.seconds()>0.2) {intake.setState(Intake.State.INTAKE_UP);intake.latch.setState("goOpen");}
                        if(intake.extendo.inPosition() && driveTrain.inPosition(800 , 600 , 1) && driveTrain.targetX==-300)intake.setState(Intake.State.REVERSE_DOWN);
                    }
                    ,
                    ()->{
                        timer.reset();
                        /**if(intake.hasSample.getState() && timerForSub.seconds()>1 && driveTrain.inPosition(800 , 600 , 1) && intake.extendo.inPosition() && Extendo.targetPosition==900)timerForSub.reset();
                        if(intake.hasSample.getState() && timerForSub.seconds()>0.1 && driveTrain.inPosition(800 , 600 , 1) && intake.extendo.inPosition() && Extendo.targetPosition==900){
                            intake.setExtendoIN();
                        return true;}**/
                        return false;
                    }
                    ,
                    new Node[]{takeFloorSample , beforeTakeWallSpecimen}
            );


            takeFloorSample.addConditions(
                    ()->{
                        if(takeSample.index==0 && !driveTrain.inPosition())intake.setExtendoIN();
                        intake.latch.setState("goOpen");
                        outtake.goDefault();
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takeFloorSample.index]);
                        if(driveTrain.inPosition(30 , 30 , 0.15) && Math.abs(Odo.odo.getHeadingVelocity())<4)
                        {intake.setExtendoTargetPosition(intakeExtendoPosition[takeFloorSample.index]);
                        }
                        else timer.reset();
                        intake.setState(Intake.State.INTAKE_DOWN);

                    }
                    ,
                    ()->{
                        return true;
                        //if(timer.seconds()>1){return true;}
                        //return !intake.hasSample.getState();
                    }
                    ,
                    new Node[]{releaseSample}

            );
            releaseSample.addConditions(
                    ()->{
                        timer.reset();
                        driveTrain.setTargetPosition(releaseSamplePosition[releaseSample.index]);
                        if(driveTrain.inPosition(150 , 150 , 0.5) || releaseSample.index>0)
                        intake.setExtendoTargetPosition(reverseExtendoPosition[releaseSample.index]);

                        if(driveTrain.inPosition(150 , 150 , 0.5))intake.setState(Intake.State.REVERSE_DOWN);
                    }
                    ,
                    ()->{
                        if(reverseTimer.seconds()>0.4 && driveTrain.inPosition(150 , 150 , 0.5) && intake.state== Intake.State.REVERSE_DOWN)reverseTimer.reset();
                        //f(reverseTimer.seconds()>0.3 && reverseTimer.seconds()<0.4 && intake.hasSample.getState() && intake.state== Intake.State.REVERSE_DOWN){intake.setExtendoTargetPosition(450);return true;}
                        return false;
                    }
                    ,
                    new Node[]{takeFloorSample , takeFloorSample , beforeTakeWallSpecimen}
            );
            beforeTakeWallSpecimen.addConditions(
                    ()->{
                        intake.setExtendoIN();

                        driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition);

                        outtake.takeSpecimen();
                        intake.setState(Intake.State.REPAUS_UP);
                    }
                    ,
                    ()->{

                        return (outtake.inPosition() && driveTrain.inPosition(150 ,100 , 0.25) && outtake.state== Outtake.State.Specimen && outtake.arm.state==outtake.arm.states.get("takeSpecimen"));
                    }
                    ,
                    new Node[]{takeWallSpecimen}
            );

            takeWallSpecimen.addConditions(
                    ()->{


                        {
                        driveTrain.setTargetPosition(takeWallSpecimenPosition);
                        if(driveTrain.inPosition(30 ,100 , 0.3))outtake.grabSample();}
                    }
                    ,
                    ()->{
                        if(sixth)takeWallSpecimen.next[4]=beforePutSpecimen;
                        if((outtake.claw.state==outtake.claw.states.get("closeSpecimen") || outtake.claw.state==outtake.claw.states.get("goCloseSpecimen")) && !a){timerOuttake.reset();return true;}
                        return false;
                    }
                    ,
                    new Node[]{beforePutSpecimen , beforePutSpecimen , beforePutSpecimen , beforePutSpecimen, beforePutSpecimen , takeWallSpecimen, takeWallSpecimen}
            );

            currentNode=beforePutSpecimen;
        }
        else {
        currentNode.run();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];



        Odo.update();

        driveTrain.update();
        intake.update();
        outtake.update();
        }

    }




}
