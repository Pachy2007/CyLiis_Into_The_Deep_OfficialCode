package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.network.ApChannel;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

@Config
public class Outtake {

    public enum State{
        GoDown ,
        Deafult ,
        TakingElement ,
        CLIMBAUTO ,
        Specimen ,
        DeafultWithElement,
        ReleaseSample,
        Up,
        PARK,
        CLIMB2,
        CLIMB3;
    }
    public State state=State.Deafult;

    enum Scoring{

        HighBasket ,
        LowBasket ,
        HighChamber, ScoringHighChamber,
        LowChamber , ScoringLowChamber
    } public Scoring scoring=Scoring.HighChamber;

    public boolean haveSample=false;
    boolean high=true;
    public boolean climb=false;

    boolean retry=false;
    public Arm arm;
    public Lift lift;
    public Claw claw;
    public Extension extension;

    boolean take=false;
    boolean sample=false;
    boolean release=false;

    boolean calculate=false;

    public static double sumChamber , sumBasket;
    public static double nrChamber , nrBasket;

    public static Pose2D scoringSamplePos;

    ElapsedTime timerForBasket=new ElapsedTime();
    ElapsedTime timerForChamber=new ElapsedTime();

    ElapsedTime retryTimer=new ElapsedTime();

    public boolean climb3ArmPosition=false;
    public static boolean prim=false;
    public static int lowBasketPosition=400  , highBasketPosition=1100  , lowChamberDown=150 ,  highChamberDown=570, climbPosition=500 , autoClimbPosition=190 , climb2=520 , climb3=1100;

    public Outtake()
    {
        extension=new Extension();
        arm=new Arm();
        lift=new Lift();
        claw=new Claw();

    }
    public Outtake(State state)
    {
        this.state=state;
        extension=new Extension();
        arm=new Arm("goWithElementSpecimen");
        lift=new Lift();
        claw=new Claw("goCloseSpecimen");
    }

    public void climb2(){
        state=State.CLIMB2;
        update();
    }

    public void climb3(){
        state=State.CLIMB3;
        update();
    }

    public void retry()
    {
        if(!haveSample && state==State.Up)retry=true;
        retryTimer.reset();
    }

    public void autoCLimb(){
        state=State.CLIMBAUTO;
    }

    public void grabSample()
    {
        if(lift.state!= Lift.State.DOWN && arm.inPosition() && extension.inPosition())return;
        switch (state)
        {
            case Deafult:
                if(arm.state!=arm.states.get("deposit") || extension.state!=extension.states.get("takeSample") || lift.state!= Lift.State.DOWN)return;
                haveSample=true;
                break;

            case Specimen:
                if(arm.inPosition() && arm.state==arm.states.get("takeSpecimen") && lift.state== Lift.State.DOWN)
                take=true;
            case DeafultWithElement:
            case TakingElement:
            case Up:
            case CLIMB2:
            case PARK:
            case GoDown:
            case ReleaseSample:
            case CLIMB3:
                return;
        }
        state=State.TakingElement;
    }

    public void releaseSample()
    {
        if(state!=State.DeafultWithElement && state!=State.ReleaseSample)return;

         if(state==State.ReleaseSample && arm.state==arm.states.get("takeSpecimen"))release=true;
        else release=false;
        state=State.ReleaseSample;
    }

    public void takeSpecimen()
    {
        if(state==State.Up || state==State.DeafultWithElement || state==State.TakingElement || state==State.Specimen)return;
        if(arm.state!=arm.states.get("highSpecimen") && arm.state!=arm.states.get("goNeutralSpecimen"))
        state=State.Specimen;
    }
    public void goDefault()
    {
        haveSample=true;
        state=State.Deafult;
    }

    public void park()
    {
        state=State.PARK;
    }

    public void goForHigh()
    {
        high=true;
    }
    public void goForLow()
    {
        high=false;
    }

    public void score(){
        if(state!=State.Up || !arm.inPosition())return;

        scoringSamplePos = new Pose2D(Odo.getX() , Odo.getY() , Odo.getHeading());
        state=State.GoDown;
    }



    public void goUp()
    {
        if(state==State.DeafultWithElement && arm.servos[0].getPosition()<0.7)
            state=State.Up;
    }

    public boolean inPosition()
    {
        return arm.inPosition() && claw.inPosition() && lift.inPosition() && extension.inPosition();
    }

    private void updateHardware()
    {
        switch (state)
        {
            case CLIMBAUTO:
                extension.setState("extend");
                arm.setState("goWithElementSpecimen");
                lift.goDown();
                break;
            case CLIMB2:
                lift.setPosition(climb2);
                lift.goUp();
                break;
            case CLIMB3:
                lift.setPosition(climb3);
                lift.goUp();
                climb3ArmPosition=true;
                break;
            case PARK:
                lift.setPosition(autoClimbPosition);
                lift.goUp();
                extension.setState("goExtend");
                arm.setState("goHighSpecimen");
                break;
            case ReleaseSample:
                calculate=false;
                arm.setState("goTakeSpecimen");
                extension.setState("goTakeSpecimen");
                if(arm.inPosition() && extension.inPosition() && release)
                    claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))state=State.Deafult;
                lift.goDown();
            break;

            case Specimen:
                calculate=false;

                if((extension.inPosition() && extension.state==extension.states.get("takeSpecimen")) || extension.state!=extension.states.get("deposit"))arm.setState("goTakeSpecimen");
                extension.setState("goTakeSpecimen");
                haveSample=false;
                if(!take && extension.inPosition())claw.setState("goTakeSpecimen");
                if(take==true && claw.state==claw.states.get("takeSpecimen"))claw.setState("goCloseSpecimen");
                if(claw.state==claw.states.get("closeSpecimen"))state=State.DeafultWithElement;
                lift.goDown();
                break;

            case TakingElement:
                if(claw.state==claw.states.get("close"))
                state=State.DeafultWithElement;

                if(arm.state==arm.states.get("deposit") && extension.state==extension.states.get("takeSample") && extension.inPosition())
                claw.setState("goClose");
                arm.setState("goDeposit");

                if(arm.inPosition() && arm.state==arm.states.get("deposit"))
                extension.setState("goTakeSample");

                calculate=false;
                lift.goDown();
                  haveSample=true;
                if( claw.state==claw.states.get("close"))
                {lift.setPosition(150);
                lift.goUp();}
                break;

            case Deafult:
                take=false;
                calculate=false;
                if(!climb3ArmPosition)
                arm.setState("goDeposit");
                else
                {
                    arm.setState("goFinalClimb3");
                }
                if(arm.inPosition() && arm.state==arm.states.get("deposit"))
                extension.setState("goTakeSample");

                else extension.setState("goRetrect");

                claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))
                        arm.setState("goDeposit");


                lift.goDown();
                break;

            case DeafultWithElement:
                calculate=false;
                extension.setState("goRetrect");

                if(haveSample)
                arm.setState("goWithElementSample");
                else arm.setState("goWithElementSpecimen");

                if(haveSample)
                claw.setState("goClose");
                else claw.setState("goCloseSpecimen");
                lift.setPosition(150);
                break;

            case GoDown:

                calculate=false;
                take=false;
                if(!haveSample && claw.state!=claw.states.get("goScoring"))
                {claw.setState("goScoring");
                extension.setState("goRetrect");}
                if(claw.inPosition() && arm.state!=arm.states.get("neutral") && arm.state!=arm.states.get("goNeutral") && arm.inPosition()){

                    if(haveSample)
                    {
                        if(arm.state==arm.states.get("highSample"))
                        if(arm.state!=arm.states.get("goNeutralSample") && arm.state!=arm.states.get("neutralSample"))
                        arm.setState("goScoreSample");
                        if(arm.state==arm.states.get("scoreSample") && arm.inPosition())claw.setState("goScoring");

                        if(arm.inPosition() && arm.state==arm.states.get("scoreSample") && claw.state==claw.states.get("scoring"))
                        {arm.setState("goNeutralSample");}
                    }
                    else if(extension.state==extension.states.get("retrect"))arm.setState("goNeutralSpecimen");
                    extension.setState("goRetrect");}
                if((claw.inPosition() && (arm.state==arm.states.get("neutralSample") && Math.sqrt( (scoringSamplePos.y-Odo.getY())*(scoringSamplePos.y-Odo.getY()) + (scoringSamplePos.x-Odo.getX())*(scoringSamplePos.x-Odo.getX()) )>100 && (scoringSamplePos.x<Odo.getX() || scoringSamplePos.y>Odo.getY())  ) || arm.state==arm.states.get("neutralSpecimen")) && claw.state==claw.states.get("scoring"))state = State.Deafult;

                break;

            case Up:
            {
                switch (scoring)
                {
                    case LowBasket:
                        lift.goUp();
                        lift.setPosition(lowBasketPosition);
                        extension.setState("goRetrect");
                        arm.setState("goLowSample");
                        break;

                    case HighBasket:
                        if(Lift.position!=highBasketPosition)timerForBasket.reset();
                        lift.goUp();
                        lift.setPosition(highBasketPosition);
                        extension.setState("goRetrect");
                        arm.setState("goHighSample");
                        break;

                    case LowChamber:
                        lift.goUp();
                        arm.setState("goHighSpecimen");
                        extension.setState("goExtend");
                        lift.setPosition(lowChamberDown);
                        break;

                    case HighChamber:
                        if(Lift.position!=highChamberDown)timerForBasket.reset();
                        lift.goUp();
                        arm.setState("goHighSpecimen");

                        if(!retry)
                        if(arm.state==arm.states.get("highSpecimen"))
                        extension.setState("goExtend");
                        if(retry)
                        {
                            extension.setState("goRetrect");
                        }
                        if(extension.state==extension.states.get("retrect"))retry=false;

                        lift.setPosition(highChamberDown);

                        break;
                }
            }
            break;
        }
    }

    private void updateUpState()
    {
        if((scoring==Scoring.ScoringHighChamber || scoring==Scoring.ScoringLowChamber) && state==State.Up)return;
        if(haveSample)
        {
            if(high)scoring=Scoring.HighBasket;
            if(!high)scoring=Scoring.LowBasket;
        }
        else{
            if(high)scoring=Scoring.HighChamber;
            if(!high)scoring=Scoring.HighChamber;
        }
    }


    public void update()
    {

        claw.update();
        if(!climb)
        lift.update();
        arm.update();
        extension.update();

        updateUpState();
        updateHardware();

        if(lift.inPosition() && state==State.Up && !calculate){
            calculate=true;
            if(scoring==Scoring.HighChamber){nrChamber++;sumChamber+=timerForChamber.seconds();}
            if(scoring==Scoring.HighBasket){nrBasket++;sumBasket+=timerForBasket.seconds();}
        }
    }
}