package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public abstract class IServoModule{

    public boolean ENABLE=true;
    public State state;


    public String moduleName;
    public double maxVelocity , acceleration , deceleration;

    public BetterServo[] servos;

    public void setServos(BetterServo... servos)
    {
        this.servos=servos;
    }

    public StateArray states=new StateArray();


    public void setProfileCoefficients()
    {
        for(int i=0;i<servos.length;i++)
            servos[i].setProfileCoefficients(maxVelocity , acceleration , deceleration);
    }

    public abstract void setStates();
    public abstract void updateStatesPosition();
    public abstract void atStart() ;


    public void updateState()
    {
         for(int i=0;i<servos.length ; i++)
             if(!   servos[i].inPosition())return;

         state=state.nextState;
    }


    public void setState(String name)
    {
        if(state.positions==states.get(name).positions)return;
        if(state==states.get(name) || state==states.get(name).nextState)return;
        state=states.get(name);
        updateHardware();
    }


    public void updateHardware()
    {
        for(int i=servos.length-1;i>=0;i--) {
            servos[i].setPosition(state.getPosition(i));
            servos[i].update();
        }
    }

    public void telemetry(Telemetry telemetry)
    {
        for(int i=0;i<servos.length;i++)
        {
            telemetry.addData( moduleName+servos[i].name , servos[i].getPosition());
        }
        telemetry.addData( moduleName+"STATE" , state.name);
        telemetry.addData("nextState" , state.nextState.name);
    }

    public boolean inPosition()
    {
        for(int i=0;i<servos.length;i++)
        {if(!servos[i].inPosition())return false;}
        return true;
    }



    public void update()
    {
        updateStatesPosition();
        updateState();
        updateHardware();
    }



}