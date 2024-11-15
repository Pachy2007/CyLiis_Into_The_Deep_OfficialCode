package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Lift {


    enum State{
        UP() , GOING_UP(UP) , DOWN() , GOING_DOWN(DOWN);
        State nextState;
        State(State nextState)
        {
            this.nextState=nextState;
        }
        State()
        {
            this.nextState=this;
        }
    }
    State state=State.DOWN;
    public BetterMotor motorLeft , motorRight;
    public static boolean motorLeftReversed=true;
    public static boolean motorRightReversed=true;
    public static int maxPosition=1600;

    public static double kP=0.0078 , kI=0.00001 , kD=0.0001;


    public static double position;
    public static double position1;
    public Lift()
    {

        motorLeft=new BetterMotor(Hardware.meh0 , BetterMotor.RunMode.PID , motorLeftReversed  , Hardware.mch2);
        motorRight=new BetterMotor(Hardware.meh1 , BetterMotor.RunMode.PID , motorRightReversed  , Hardware.mch2);

        motorLeft.setPidCoefficients(kP , kD , kI);
        motorRight.setPidCoefficients(kP , kI , kD);

    }


    public void goUp()
    {
        state=State.GOING_UP;
    }
    public void goDown()
    {
        if(state!=State.GOING_DOWN && state!=State.DOWN)
            state=State.GOING_DOWN;
    }
    public void setPosition(int position)
    {
        this.position=position;
        this.position=Math.min(maxPosition , position);
    }
    public void increasePosition(int extra)
    {
        position+=extra;
        position=Math.min(maxPosition , position);
    }

    public boolean inPosition()
    {
        if(Math.abs(position- motorLeft.getPosition())<15)return true;
        return false;
    }
    public void decreasePosition(int extra)
    {
        position-=extra;
        position=Math.max(150 , position);
    }
    private void updateState()
    {
        switch(state)
        {
            case GOING_UP:
                if(Math.abs(position- motorLeft.getPosition())<70)state=state.nextState;
                break;
            case UP:
            case DOWN:
                break;
            case GOING_DOWN:
                if(Math.abs(motorLeft.getVelocity())<0.001 && motorLeft.getPosition()<30)
                {state=state.nextState; motorLeft.resetPosition(); motorRight.resetPosition();}
                break;
        }
    }
    private void updateHardware()
    {
        switch(state)
        {
            case GOING_UP:
            case UP:
                motorLeft.setPosition(position);
                motorRight.setPosition(position);
                break;
            case DOWN:
                motorLeft.setPower(0);
                motorRight.setPower(0);
                break;
            case GOING_DOWN:
                motorLeft.setPower(-1);
                motorRight.setPower(-1);
                break;
        }
    }

    private void updateCoefficient()
    {
        motorLeft.setPidCoefficients(kP , kD , kI);
        motorRight.setPidCoefficients(kP , kI , kD);
    }

    public void update()
    {
        updateCoefficient();
        updateState();
        updateHardware();
        motorRight.update();
        motorLeft.update();
    }


}