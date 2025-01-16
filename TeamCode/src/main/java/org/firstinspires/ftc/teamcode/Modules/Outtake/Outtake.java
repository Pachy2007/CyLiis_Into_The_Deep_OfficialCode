package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.network.ApChannel;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.opencv.core.Mat;

@Config
public class Outtake {

    public enum State{
        GoDown ,
        Deafult ,
        TakingElement ,
        Specimen ,
        DeafultWithElement,
        ReleaseSample,
        Up,
        PARK
    }
    public State state=State.Deafult;

    enum Scoring{

        HighBasket ,
        LowBasket ,
        HighChamber, ScoringHighChamber,
        LowChamber , ScoringLowChamber
    } public Scoring scoring=Scoring.LowChamber;

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

    ElapsedTime retryTimer=new ElapsedTime();

    public static boolean prim=false;
    public static int lowBasketPosition=400  , highBasketPosition=1100  , lowChamberDown=150 ,  highChamberDown=510, climbPosition=500 , autoClimbPosition=190;

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


    public void retry()
    {
        if(!haveSample && state==State.Up)retry=true;
        retryTimer.reset();
    }

    public void grabSample()
    {
        if(lift.state!= Lift.State.DOWN)return;
        switch (state)
        {
            case Deafult:
                arm.setState("goDeposit");
                haveSample=true;
                break;

            case Specimen:take=true;
            case DeafultWithElement:
            case TakingElement:
            case Up:
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

            sample=true;
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
            case PARK:
                lift.setPosition(autoClimbPosition);
                lift.goUp();
                extension.setState("goExtend");
                arm.setState("goHighSpecimen");
                break;
            case ReleaseSample:
                arm.setState("goTakeSpecimen");
                extension.setState("goTakeSpecimen");
                if(arm.inPosition() && extension.inPosition() && release)
                    claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))state=State.Deafult;
                lift.goDown();
            break;

            case Specimen:

                if((extension.inPosition() && extension.state==extension.states.get("takeSpecimen")) || extension.state!=extension.states.get("deposit"))arm.setState("goTakeSpecimen");
                extension.setState("goTakeSpecimen");
                haveSample=false;
                if(!take && extension.inPosition())claw.setState("goTakeSpecimen");
                if(take==true && claw.state==claw.states.get("takeSpecimen"))claw.setState("goCloseSpecimen");
                if(claw.state==claw.states.get("closeSpecimen"))state=State.DeafultWithElement;
                lift.goDown();
                break;

            case TakingElement:
            lift.goDown();
                {   haveSample=true;
                    if(arm.inPosition() && extension.inPosition())
                    extension.setState("goDeposit");
                    if(arm.inPosition() && lift.state==Lift.State.DOWN && extension.inPosition() && extension.state==extension.states.get("deposit"))claw.setState("goClose");
                if(claw.inPosition() && claw.state==claw.states.get("close"))state=State.DeafultWithElement;}
                if(claw.inPosition() && claw.state==claw.states.get("close"))
                {lift.setPosition(150);
                lift.goUp();}
                break;

            case Deafult:
                take=false;


                if(arm.inPosition() && arm.state==arm.states.get("deposit"))
                extension.setState("goTakeSample");

                else extension.setState("goRetrect");

                claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))
                        arm.setState("goDeposit");
                 if(arm.inPosition() && arm.state==arm.states.get("deposit"))

                 if(!climb)
                lift.goDown();
                 else {lift.setPosition(climbPosition);lift.goUp();}
                break;

            case DeafultWithElement:
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

                take=false;
                if(!haveSample)
                claw.setState("goScoring");
                extension.setState("goRetrect");
                if(claw.inPosition() && arm.state!=arm.states.get("neutral") && arm.state!=arm.states.get("goNeutral") && arm.inPosition()){

                    if(haveSample)
                    {
                        if(arm.state!=arm.states.get("goNeutralSample") && arm.state!=arm.states.get("neutralSample"))
                        arm.setState("goScoreSample");
                        arm.maxVelocity=16;
                        if(arm.state==arm.states.get("scoreSample"))claw.setState("goScoring");

                        if(claw.state==claw.states.get("scoring"))
                        {arm.maxVelocity=20; arm.setState("goNeutralSample");}
                    }
                    else arm.setState("goNeutralSpecimen");
                    extension.setState("goRetrect");}
                if(claw.inPosition() && (arm.state==arm.states.get("neutralSample") || arm.state==arm.states.get("neutralSpecimen")) && claw.state==claw.states.get("scoring"))lift.goDown();
                if(lift.state== Lift.State.DOWN)
                    state = State.Deafult;

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
                        lift.goUp();
                        lift.setPosition(highBasketPosition);
                        extension.setState("goRetrect");
                        arm.setState("goHighSample");
                        break;

                    case LowChamber:
                        lift.goUp();
                        arm.setState("goLowSpecimen");
                        extension.setState("goExtend");
                        lift.setPosition(lowChamberDown);
                        break;

                    case HighChamber:
                        lift.goUp();
                        arm.setState("goHighSpecimen");

                        if(!retry)
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
        lift.update();
        arm.update();
        extension.update();

        updateUpState();
        updateHardware();
    }
}