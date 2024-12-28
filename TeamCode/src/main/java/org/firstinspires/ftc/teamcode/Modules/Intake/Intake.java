package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {


    public enum State{
        REPAUS_DOWN , REPAUS_UP , INTAKE_UP , INTAKE_DOWN , REVERSE_UP , REVERSE_DOWN
    }
    public State state;
    public ActiveIntake activeIntake;
    DropDown dropDown;
    public Ramp ramp;

    ElapsedTime timer=new ElapsedTime();

    public Intake()
    {
        ramp=new Ramp();
        activeIntake=new ActiveIntake();
        dropDown=new DropDown();
        timer.startTime();
        state=State.REPAUS_UP;

    }

    private void updateStates()
    {
        switch (state)
        {
            case REPAUS_UP:
                ramp.setState("goUp");
                dropDown.setState("goRampUp");
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                break;
            case INTAKE_UP:
                ramp.setState("goUp");
                dropDown.setState("goTakeElement");
                activeIntake.setMode(ActiveIntake.State.INTAKE);
                break;
            case REVERSE_UP:
                ramp.setState("goUp");
                dropDown.setState("goTakeElement");
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                break;
            case REPAUS_DOWN:
                ramp.setState("goDown");
                dropDown.setState("goRampDown");
                activeIntake.setMode(ActiveIntake.State.REPAUS);
                break;
            case INTAKE_DOWN:
                ramp.setState("goDown");
                dropDown.setState("goTakeElement");
                activeIntake.setMode(ActiveIntake.State.INTAKE);

                if(activeIntake.motor.motor.isOverCurrent())
                {
                    timer.reset();
                }
                if(timer.seconds()<0.5)dropDown.setState("goJam");

                break;
            case REVERSE_DOWN:
                ramp.setState("goDown");
                dropDown.setState("goTakeElement");
                activeIntake.setMode(ActiveIntake.State.REVERSE);
                break;
        }
    }

    public boolean inPosition()
    {
       if(ramp.inPosition() && dropDown.inPosition())return true;
        return false;
    }

    public void setState(State state) {
        this.state=state;
    }

    public void update()
    {
        ramp.update();
        dropDown.update();
        updateStates();
    }

}