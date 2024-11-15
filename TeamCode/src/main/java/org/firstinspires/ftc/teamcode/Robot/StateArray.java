package org.firstinspires.ftc.teamcode.Robot;

import java.util.ArrayList;

public class StateArray {

    ArrayList<State> states=new ArrayList<>();

    public void addState(String name , State nextState , double...positions){
        states.add(new State(name , nextState , positions));
    }
    public void addState(String name ,double...positions){
        states.add(new State(name ,positions));
    }

    public State get(String name) {
        for(int i=0 ; i<states.size();i++)
            if(states.get(i).verify(name))return states.get(i);

      return null;
    }


}