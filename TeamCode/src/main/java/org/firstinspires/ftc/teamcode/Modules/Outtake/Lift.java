package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;
import org.opencv.core.Mat;

@Config
public class Lift {


    public enum State{
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
    public State state=State.GOING_DOWN;
    public static int maxPosition=1600;
    public double prevVelocity=0;
    public static boolean climb=false;
    public static boolean yes=false;
    int nr=0;

    public static double kP=0.011 , kI=0 , kD=0.00022;

    PIDController controller;
    Encoder encoder;

    public static double position;
    public static double position1;
    public static boolean encoderReversed=false;
    public Lift()
    {
        Differential.init();
        controller=new PIDController(kP , kI , kD);
        encoder=new Encoder(Hardware.mch3 , encoderReversed);
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
    public void setPosition(double position)
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
        if(climb==true) {if((Math.abs(position- encoder.getPosition())<100) || state==State.DOWN || state==State.GOING_DOWN )return true;}
        else if(Math.abs(position- encoder.getPosition())<35 || state==State.DOWN || state==State.GOING_DOWN)return true;
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
                if(Math.abs(position+ encoder.getPosition())<70)state=state.nextState;
                nr=0;
                break;
            case UP:
            case DOWN:
                nr=0;
                break;
            case GOING_DOWN:
                if((encoder.getVelocity()<0.00000000001 && climb && encoder.getPosition()<20) || (Math.abs(encoder.getVelocity())<0.01 && !climb))
                {   nr++;
                    if(nr>=2)
                    {state=state.nextState; encoder.resetPosition();}}
                break;
        }
    }
    private void updateHardware()
    {
        switch(state)
        {
            case GOING_UP:
            case UP:
                double power=controller.calculate(position , encoder.getPosition());

                Differential.liftPower=power;
                break;
            case DOWN:
                Differential.liftPower=-0;
                //motorLeft.setPower(-0.05);
                //motorRight.setPower(-0.05);
                break;
            case GOING_DOWN:
                Differential.liftPower=-1;
                //motorLeft.setPower(-1);
                //motorRight.setPower(-1);
                break;
        }
    }

    private void updateCoefficient()
    {
        controller.kp=kP;
        controller.ki=kI;
        controller.kd=kD;
    }

    public void update()
    {

        updateHardware();
        Differential.update();
        updateCoefficient();
        updateState();
        prevVelocity=encoder.getVelocity();
    }


}