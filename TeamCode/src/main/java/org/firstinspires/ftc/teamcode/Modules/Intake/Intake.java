package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Intake {


    public enum State{
        REPAUS_DOWN , REPAUS_UP , INTAKE_UP , INTAKE_DOWN , REVERSE_UP , REVERSE_DOWN;
    }

    public enum TransferLogic{
        Free , Decide , Throw , Transfer , ReadyToTransfer , PrepareToHuman , ReadyForHuman;
    }

    public enum Asure1inDeposit{
        Free , PrepareToClean , Clean , DezactivateCleaning , Done;
    }

    public Asure1inDeposit asure1inDepositState=Asure1inDeposit.Free;

    public TransferLogic stateTransfer=TransferLogic.Free;


    public State state=State.REPAUS_UP , prevState=State.REPAUS_UP;
    public State driverState=State.REPAUS_UP;

    public ActiveIntake activeIntake;
    public Ramp ramp;
    public Extendo extendo;
    public ExtendoBlocker extendoBlocker;
    public Latch latch;

    public SampleColor sampleColor;
    public static DigitalChannel bbDeposit;

    public SampleColor.State colorAllaiance;
    ElapsedTime decideTimer = new ElapsedTime();

    public boolean sampleInDeposit;
    public double kNR=0;
    public boolean justColorAllaiance=false;
    public boolean transfer=true;

    boolean usingAutomatics=false;
    int nr=0;
    boolean having;
    int reverseTicks=0;


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
        bbDeposit= Hardware.depositBeamBreak;




    }
    private void updateDeposit()
    {
        switch (stateTransfer)
        {
            case Free:
                if(sampleInDeposit)
                    stateTransfer=TransferLogic.Decide;
                decideTimer.reset();
            break;
            case Decide:
                if(!sampleInDeposit)stateTransfer=TransferLogic.Free;
                if(decideTimer.seconds()<0.1)return;

                if(sampleColor.state!=colorAllaiance && sampleColor.state== SampleColor.State.YELLOW)stateTransfer=TransferLogic.Throw;
                if(sampleColor.state== SampleColor.State.YELLOW)
                {
                    if(justColorAllaiance)stateTransfer=TransferLogic.Throw;
                    if(!justColorAllaiance)stateTransfer=TransferLogic.Transfer;
                }
                if(sampleColor.state==colorAllaiance)
                {
                    if(justColorAllaiance && !transfer)stateTransfer=TransferLogic.PrepareToHuman;
                    else stateTransfer=TransferLogic.Transfer;
                }
                if(sampleColor.state!=colorAllaiance && sampleColor.state!= SampleColor.State.YELLOW)stateTransfer=TransferLogic.Throw;
            break;
            case Throw:
                state=State.REVERSE_DOWN;
                kNR++;
                if(kNR>=5){kNR=0; stateTransfer=TransferLogic.Free;}
            break;
            case PrepareToHuman:
                if(asure1inDepositState==Asure1inDeposit.Free)asure1inDepositState=Asure1inDeposit.PrepareToClean;

                if(asure1inDepositState==Asure1inDeposit.Done){asure1inDepositState=Asure1inDeposit.Free;stateTransfer=TransferLogic.ReadyForHuman;}
            break;

            case ReadyForHuman:
                if(!sampleInDeposit && extendo.state== Extendo.State.GOING_IN)stateTransfer=TransferLogic.Free;
                if(prevState==State.REVERSE_UP && state==State.REPAUS_UP){extendo.setIn();stateTransfer=TransferLogic.Free;}
                prevState=state;
            break;

            case Transfer:
                if(asure1inDepositState==Asure1inDeposit.Free)asure1inDepositState=Asure1inDeposit.PrepareToClean;

                if(asure1inDepositState==Asure1inDeposit.Done){asure1inDepositState=Asure1inDeposit.Free;stateTransfer=TransferLogic.ReadyToTransfer;}
            break;
            case ReadyToTransfer:
                state=State.INTAKE_UP;
                if(!sampleInDeposit)stateTransfer=TransferLogic.Free;
            break;
        }
    }


    public void setExtendoVelocity(double velocity)
    {
        if(stateTransfer==TransferLogic.Free || stateTransfer==TransferLogic.ReadyForHuman)
        if(Math.abs(velocity)>0.07 && !having && !transfer)state=State.INTAKE_UP;
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

    private void updateAsure1inDeposit()
    {
        switch (asure1inDepositState)
        {
            case Free:
            break;
            case PrepareToClean:
                latch.setState("goClose");
                reverseTicks=0;
                if(latch.inPosition())
                {extendo.setIn();state=State.REVERSE_UP;}
                else state=State.INTAKE_UP;
                if(latch.inPosition() && ramp.servos[0].getPosition()>Ramp.upPosition-0.35)asure1inDepositState=Asure1inDeposit.Clean;
            break;
            case Clean:
                ActiveIntake.State.REVERSE.power=ActiveIntake.reverPowerTransfer;
                state=State.REVERSE_UP;
                reverseTicks++;
                extendo.setIn();
                if(reverseTicks>=3){asure1inDepositState=Asure1inDeposit.DezactivateCleaning;reverseTicks=0;}
            break;
            case DezactivateCleaning:
                extendo.setIn();
                state=State.INTAKE_UP;
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                latch.setState("goOpen");
                if(latch.inPosition())
                asure1inDepositState=Asure1inDeposit.Done;
            break;
            case Done:
                setState(State.INTAKE_UP);
                extendo.setIn();
            break;
        }
    }

    private void updateStates()
    {
        if(stateTransfer==TransferLogic.Free || stateTransfer==TransferLogic.ReadyForHuman){
            if(driverState==State.REVERSE_DOWN && stateTransfer==TransferLogic.ReadyForHuman)
            state=State.REVERSE_UP;
            else if(asure1inDepositState==Asure1inDeposit.Free)state=driverState;

        }

        if(driverState==State.REVERSE_DOWN){state=driverState; asure1inDepositState=Asure1inDeposit.Free;}

        switch (state)
        {
            case INTAKE_UP:
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                ramp.setState("goUp");
            break;
            case REPAUS_UP:
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                ramp.setState("goUp");
            break;
            case INTAKE_DOWN:
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                ramp.setState("goDown");
            break;
            case REVERSE_UP:
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                ramp.setState("goUp");
            break;
            case REPAUS_DOWN:
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                ramp.setState("goDown");
            break;
            case REVERSE_DOWN:
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                ramp.setState("goDown");
            break;
        }


    }

    public boolean inPosition()
    {
       return ramp.inPosition();
    }

    public void setState(State state) {
        driverState=state;
    }

    public void update()
    {
        if(extendo.state== Extendo.State.IN)extendoBlocker.setState("goClose");
        else extendoBlocker.setState("goOpen");




        sampleInDeposit=(!bbDeposit.getState());
        sampleColor.update();
        ramp.update();
        extendo.update();
        extendoBlocker.update();

        latch.update();

        if(usingAutomatics)
        updateDeposit();
        updateStates();
        updateAsure1inDeposit();

    }

}