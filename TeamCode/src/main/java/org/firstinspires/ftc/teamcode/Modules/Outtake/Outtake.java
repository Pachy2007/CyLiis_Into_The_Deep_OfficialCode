package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.network.ApChannel;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Imu;
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

    boolean fromFront=false;
    boolean scoringFromFront=false;

    public static int lowBasketPosition=650  , highBasketPosition=1280 , lowChamberUp=150 , lowChamberDown=150 , highChamberUp=650 , highChamberDown=300;

    public Outtake()
    {
        arm=new Arm();
        lift=new Lift();
        claw=new Claw();
    }


    public void grabSample()
    {
        switch (state)
        {
            case Deafult:
                arm.setState("goDeposit");
                haveSample=true;
                break;

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
        if(state==State.Up || state==State.DeafultWithElement || state==State.TakingElement)return;
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
            state=State.GoDown;
        }

        if(scoring== Scoring.LowChamber)scoring=Scoring.ScoringLowChamber;
        if(scoring==Scoring.HighChamber)scoring=Scoring.ScoringHighChamber;
    }


    public void goUp()
    {
        if(state==State.DeafultWithElement)
            state=State.Up;
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
                if(arm.state==arm.states.get("default"))
                    arm.setState("goBeforeTakeSpecimen");
                haveSample=false;
                if(arm.state==arm.states.get("beforeTakeSpecimen"))claw.setState("goTakeSpecimen");
                if(claw.state==claw.states.get("takeSpecimen"))arm.setState("goTakeSpecimenBack");
                if(arm.state==arm.states.get("takeSpecimenBack")  && claw.state==claw.states.get("takeSpecimen"))claw.setState("goClose");
                if(claw.state==claw.states.get("close"))state=State.DeafultWithElement;

                lift.goDown();
                break;
            case TakingElement:
                {if(arm.inPosition())claw.setState("goClose");
                if(claw.inPosition() && claw.state==claw.states.get("close"))state=State.DeafultWithElement;}
                if(claw.inPosition() && claw.state==claw.states.get("close"))
                {lift.setPosition(50);
                lift.goUp();}
                break;
            case Deafult:
                if(claw.inPosition() && claw.state==claw.states.get("open"))
                    if(arm.state!=arm.states.get("goDefault") && arm.state!=arm.states.get("default"))
                        arm.setState("goDefault");
                    claw.setState("goOpen");
                 if(arm.inPosition() && arm.state==arm.states.get("default"))
                lift.goDown();
                break;
            case DeafultWithElement:
                arm.setState("goWithElement");
                if(haveSample)
                claw.setState("goClose");
                else claw.setState("goCloseSpecimen");
                lift.setPosition(50);
                break;
            case GoDown:
                claw.setState("goOpen");
                if(claw.inPosition() && arm.state!=arm.states.get("neutral") && arm.state!=arm.states.get("goNeutral"))arm.setState("goNeutral");
                if(claw.inPosition() && arm.state==arm.states.get("neutral") && claw.state==claw.states.get("open"))lift.goDown();
                if(lift.state== Lift.State.DOWN)
                    state = State.Deafult;

                break;
            case Up:
            {
                double angle=Imu.getHeading();

                angle=(2*Math.PI+angle)- Math.floor((2*Math.PI+angle)/(Math.PI*2))*Math.PI*2;

                if(haveSample==true) {
                    if(angle<Math.PI*1/4 || angle>Math.PI*5/4)
                        scoringFromFront=false;
                    else scoringFromFront=true;
                }
                else{
                    if(angle<Math.PI/2 || angle>Math.PI*3/2)scoringFromFront=false;
                    else scoringFromFront=true;
                }
                switch (scoring)
                {
                    case LowBasket:
                        lift.goUp();
                        lift.setPosition(lowBasketPosition);
                        if(scoringFromFront==false)arm.setState("goLowSampleFront");
                        else arm.setState("goLowSampleBack");
                        break;
                    case HighBasket:
                        lift.goUp();
                        lift.setPosition(highBasketPosition);
                        if(scoringFromFront==false)arm.setState("goHighSampleFront");
                        else arm.setState("goHighSampleBack");
                        break;
                    case LowChamber:
                        lift.goUp();
                        if(scoringFromFront==true)arm.setState("goLowSpecimenFront");
                        else arm.setState("goLowSpecimenBack");

                        if(scoringFromFront==true)
                        {
                            if(fromFront==true)lift.setPosition(lowChamberUp);
                            else lift.setPosition(lowChamberDown);
                        }
                        else
                        {
                            if(fromFront==true)lift.setPosition(lowChamberDown);
                            else lift.setPosition(lowChamberUp);
                        }
                        break;
                    case HighChamber:
                        lift.goUp();
                        if(scoringFromFront==true)arm.setState("goHighSpecimenFront");
                        else arm.setState("goHighSpecimenBack");

                        if(scoringFromFront==true)
                        {
                            if(fromFront==true)lift.setPosition(highChamberUp);
                            else lift.setPosition(highChamberDown);
                        }
                        else
                        {
                            if(fromFront==true)lift.setPosition(highChamberDown);
                            else lift.setPosition(highChamberUp);
                        }
                        break;
                    case ScoringLowChamber:
                        if(scoringFromFront==true)
                        {
                            if(fromFront==true)lift.setPosition(lowChamberDown);
                            else lift.setPosition(lowChamberUp);
                        }
                        else
                        {
                            if(fromFront==true)lift.setPosition(lowChamberUp);
                            else lift.setPosition(lowChamberDown);
                        }
                        if(lift.inPosition())state=State.GoDown;

                        break;
                    case ScoringHighChamber:
                        if(scoringFromFront==true)
                        {
                            if(fromFront==true)lift.setPosition(highChamberDown);
                            else lift.setPosition(highChamberUp);
                        }
                        else
                        {
                            if(fromFront==true)lift.setPosition(highChamberUp);
                            else lift.setPosition(highChamberDown);
                        }
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
            if(!high)scoring=Scoring.LowChamber;
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