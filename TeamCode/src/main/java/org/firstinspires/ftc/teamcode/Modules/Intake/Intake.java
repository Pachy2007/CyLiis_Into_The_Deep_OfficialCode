package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Intake {


    public enum State{
        REPAUS_DOWN , REPAUS_UP , INTAKE_UP , INTAKE_DOWN , REVERSE_UP , REVERSE_DOWN, PREPARE_TO_TRANSFER, TRANSFER, THROW;
    }
    public State state;
    public ActiveIntake activeIntake;
    public Ramp ramp;
    public Extendo extendo;
    public ExtendoBlocker extendoBlocker;
    public Latch latch;

    public SampleColor sampleColor;
    public DigitalChannel hasSample;

    SampleColor.State colorAllaiance;
    ElapsedTime timer=new ElapsedTime();

    public double kNR=0;
    public boolean justColorAllaiance=false;
    public boolean prevHasSample=false;

    boolean usingAutomatics=false;
    int nr=0;
    boolean having;
    int reverseTicks=0;

    ElapsedTime havingTimer=new ElapsedTime();

    public Intake(SampleColor.State state , boolean usingAutomatics)
    {
        this.usingAutomatics=usingAutomatics;
        colorAllaiance=state;
        this.state=State.REPAUS_UP;

        ramp=new Ramp();
        activeIntake=new ActiveIntake();
        extendo=new Extendo();
        extendoBlocker=new ExtendoBlocker();
        latch=new Latch();
        sampleColor=new SampleColor();
        hasSample= Hardware.depositBeamBreak;




    }
    private void updateDeposit()
    {
        if(having==true && !hasSample.getState() && !justColorAllaiance)having=false;
        if(havingTimer.seconds()>0.1 && havingTimer.seconds()<0.2){having=false;extendo.setIn();}
        if(!usingAutomatics || having )return;
        if(!hasSample.getState() && ((state!=State.PREPARE_TO_TRANSFER && state!=State.TRANSFER && state!=State.THROW)))
        {
            activeIntake.setMode(ActiveIntake.State.INTAKE);
            if(timer.seconds()>0.14 && ( !justColorAllaiance || (extendo.state!= Extendo.State.IN || sampleColor.state!=colorAllaiance)))
                timer.reset();
            having=false;
            kNR=0;
        }

        if(!hasSample.getState() && timer.seconds()>0.1 && timer.seconds()<0.14 && ((extendo.state!= Extendo.State.IN || sampleColor.state!=colorAllaiance) || !justColorAllaiance))
        {
            if(sampleColor.state==colorAllaiance || (sampleColor.state== SampleColor.State.YELLOW && !justColorAllaiance))
            {
                kNR = 0;
            activeIntake.setMode(ActiveIntake.State.INTAKE);
                state=state.PREPARE_TO_TRANSFER;
            }
            else state=state.THROW;

        }
    }

    public void transfer()
    {
        state=State.PREPARE_TO_TRANSFER;
    }

    public void setExtendoVelocity(double velocity)
    {
        if(state!=State.THROW && state!=State.TRANSFER && state!=State.PREPARE_TO_TRANSFER)
            if(ramp.state==ramp.states.get("up"))
            if(Math.abs(velocity)>0.05 && !having)state=State.INTAKE_UP;
        extendo.setVelocity(velocity);
    }
    public void setExtendoTargetPosition(double position)
    {
        extendo.setTargetPosition(position);
    }

    public void setExtendoIN()
    {
        extendo.setIn();
    }



    private void updateStates()
    {
        switch (state)
        {
            case REPAUS_UP:
                ramp.setState("goUp");
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                if(usingAutomatics)
                latch.setState("goOpen");
                break;
            case INTAKE_UP:
                ramp.setState("goUp");
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                if(usingAutomatics)
                    latch.setState("goOpen");
                break;
            case REVERSE_UP:
                ramp.setState("goUp");
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                if(usingAutomatics)
                    latch.setState("goOpen");
                break;
            case REPAUS_DOWN:
                ramp.setState("goDown");
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                if(usingAutomatics)
                    latch.setState("goOpen");
                break;
            case INTAKE_DOWN:
                ramp.setState("goDown");
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                if(usingAutomatics)
                    latch.setState("goOpen");
                break;
            case REVERSE_DOWN:
                ramp.setState("goDown");
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                if(usingAutomatics)
                    latch.setState("goOpen");
                break;
            case THROW:
                latch.setState("goOpen");
                ramp.setState("goDown");
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;

                if(hasSample.getState())nr++;
                if(nr>=5){state=state.REPAUS_UP;nr=0;latch.setState("goOpen");
                }
            break;
            case PREPARE_TO_TRANSFER:
                having=true;
                if(reverseTicks<=3)
                latch.setState("goClose");

                if(latch.state==latch.states.get("close") && reverseTicks<=3){activeIntake.setMode(ActiveIntake.State.REVERSE);
                                                            ramp.setState("goUp");
                                                           reverseTicks++;
                                                           ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;}
                if(reverseTicks>3){

                    activeIntake.setMode(ActiveIntake.State.REPAUS);
                    extendo.setIn();
                    reverseTicks=0;
                    latch.setState("goOpen");
                    state=State.TRANSFER;
                    kNR=0;
                }
            break;
            case TRANSFER:
                kNR++;
                if((extendo.state== Extendo.State.GOING_IN || !justColorAllaiance) && latch.state==latch.states.get("open"))activeIntake.setMode(ActiveIntake.State.INTAKE);

                if(extendo.state== Extendo.State.IN && justColorAllaiance){state=State.REPAUS_UP;}
            break;




        }

        if(extendo.state== Extendo.State.IN)extendoBlocker.setState("goClose");
        else extendoBlocker.setState("goOpen");


    }

    public boolean inPosition()
    {
       if(ramp.inPosition())return true;
        return false;
    }

    public void setState(State state) {
        if(((this.state==State.TRANSFER && ( (justColorAllaiance && state!=State.REVERSE_DOWN) || (!justColorAllaiance  && ((!hasSample.getState() || kNR<5) || extendo.state!= Extendo.State.IN || ramp.state!=ramp.states.get("up")))))  || this.state==State.THROW || this.state==State.PREPARE_TO_TRANSFER))return;
        if(having && state==State.REVERSE_DOWN){havingTimer.reset();
        this.state=state;}
        else if(!justColorAllaiance || !having){this.state=state;having=false;}


    }

    public void update()
    {
        if(usingAutomatics)
        updateDeposit();
        updateStates();


        ramp.update();
        extendo.update();
        extendoBlocker.update();
        sampleColor.update();
        latch.update();

        prevHasSample=hasSample.getState();
    }

}