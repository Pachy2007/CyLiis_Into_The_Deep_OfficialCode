package org.firstinspires.ftc.teamcode.Robot;

public class State {

    public String name;

    public double[] positions;
    public State nextState;
    public boolean next;

    public State(String name , State nextState , double... positions)
    {
        this.name=name;
        this.nextState=nextState;
        this.positions=positions;
        next=true;
    }

    public State(String name   , double... positions)
    {
        this.positions=positions;
        this.name = name;
        this.nextState=this;
        next=false;
    }

    public double getPosition( int index) {
        return positions[index];
    }

    public boolean verify(String name)
    {
        if(this.name==name)return true;
        return false;
    }


    public void updatePositions(double... positions)
    {
        this.positions=positions;
    }

}