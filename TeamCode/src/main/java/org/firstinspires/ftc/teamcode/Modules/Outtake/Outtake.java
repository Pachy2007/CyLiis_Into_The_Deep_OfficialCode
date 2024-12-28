package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

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
        Up
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

    public Arm arm;
    public Lift lift;
    public Claw claw;
    boolean take=false;

    boolean fromFront=false;
    boolean scoringFromFront=false;
    boolean sample=false;

    public static boolean prim=false;
    public static int lowBasketPosition=730  , highBasketPosition=1300 , lowChamberUp=150 , lowChamberDown=150 , highChamberUp=700 , highChamberDown=365;

    public Outtake()
    {
        arm=new Arm();
        lift=new Lift();
        claw=new Claw();
    }
    public Outtake(State state)
    {
        this.state=state;
        arm=new Arm("goWithElement");
        lift=new Lift();
        claw=new Claw("goCloseSpecimen");
    }


    public void grabSample()
    {
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
        if(state==State.DeafultWithElement)
        state=State.ReleaseSample;
    }

    public void takeSpecimen()
    {
        if(state==State.Up || state==State.DeafultWithElement || state==State.TakingElement || state==State.Specimen)return;
        claw.setState("goOpen");
        state=State.Specimen;
    }
    public void goDefault()
    {
        haveSample=true;
        state=State.Deafult;
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
        if(scoring==Scoring.LowBasket || scoring==Scoring.HighBasket)
        {
            sample=true;
            state=State.GoDown;
        }

        if(scoring== Scoring.LowChamber)scoring=Scoring.ScoringHighChamber;
        if(scoring==Scoring.HighChamber)scoring=Scoring.ScoringHighChamber;
    }


    public void goUp()
    {
        if(state==State.DeafultWithElement)
            state=State.Up;
    }

    public boolean inPosition()
    {
        return arm.inPosition() && claw.inPosition() && lift.inPosition();
    }


    private void updateHardware()
    {
        switch (state)
        {
            case ReleaseSample:
                arm.setState("goTakeSpecimenBack");
                if(arm.state==arm.states.get("takeSpecimenBack"))
                    claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))state=State.Deafult;
                lift.goDown();
            break;
            case Specimen:
                    arm.setState("goTakeSpecimenBack");
                haveSample=false;
                if(arm.inPosition() && take==false && arm.state==arm.states.get("takeSpecimenBack"))claw.setState("goTakeSpecimen");
                if(take==true && claw.state==claw.states.get("takeSpecimen"))claw.setState("goCloseSpecimen");
                if(claw.state==claw.states.get("closeSpecimen"))state=State.DeafultWithElement;

                lift.goDown();
                break;
            case TakingElement:
            lift.goDown();
                {   haveSample=true;
                    if(arm.inPosition() && lift.state==Lift.State.DOWN)claw.setState("goClose");
                if(claw.inPosition() && claw.state==claw.states.get("close"))state=State.DeafultWithElement;}
                if(claw.inPosition() && claw.state==claw.states.get("close"))
                {lift.setPosition(150);
                lift.goUp();}
                break;
            case Deafult:
                take=false;
                claw.setState("goOpen");
                if(claw.state==claw.states.get("open"))
                        arm.setState("goDefault");
                 if(arm.inPosition() && arm.state==arm.states.get("default"))
                lift.goDown();
                break;
            case DeafultWithElement:
                arm.setState("goWithElement");
                if(haveSample)
                claw.setState("goClose");
                else claw.setState("goCloseSpecimen");
                lift.setPosition(150);
                break;
            case GoDown:

                if(sample){sample=false; arm.setState("goBuruAdvice");arm.update();}

                take=false;
                claw.setState("goOpen");
                if(claw.inPosition() && arm.state!=arm.states.get("neutral") && arm.state!=arm.states.get("goNeutral") && arm.inPosition())arm.setState("goNeutral");
                if(claw.inPosition() && arm.state==arm.states.get("neutral") && claw.state==claw.states.get("open"))lift.goDown();
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
                        arm.setState("goLowSampleBack");
                        break;
                    case HighBasket:
                        lift.goUp();
                        lift.setPosition(highBasketPosition);
                         arm.setState("goHighSampleBack");
                        break;
                    case LowChamber:
                        lift.goUp();
                        arm.setState("goLowSpecimenFront");

                        lift.setPosition(lowChamberDown);

                        break;
                    case HighChamber:
                        lift.goUp();
                        if(!prim)
                            arm.setState("goHighSpecimenFront");
                        else arm.setState("goHighSpecimenBack");

                        lift.setPosition(highChamberDown);

                        break;
                    case ScoringLowChamber:
                        lift.setPosition(lowChamberUp);

                        if(lift.inPosition())state=State.GoDown;

                        break;
                    case ScoringHighChamber:

                        lift.setPosition(highChamberUp);

                        if(lift.inPosition())state=State.GoDown;

                        break;

                }
            }break;
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
    private void updateSpecimen()
    {
        if(state!=State.Specimen)return;
        fromFront=false;
    }


    public void update()
    {

        claw.update();
        lift.update();
        arm.update();

        updateSpecimen();
        updateUpState();
        updateHardware();
    }
}