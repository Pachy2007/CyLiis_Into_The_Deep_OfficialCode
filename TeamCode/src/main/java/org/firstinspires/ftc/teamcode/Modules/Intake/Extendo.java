package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

@Config
public class Extendo {

    public enum State{
        IN , GOING_IN(IN) , OUT(GOING_IN) , GO_TO_POSITION();

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
    public State state;


    int nr=0;
    public PIDController controller;
    public Encoder encoder;
    public static boolean encoderReversed=true;
    public static double kp=0.007 , ki , kd;
    public static double targetPosition;

    private static double inPower=-0.02 , goingInPower=-1;

    public static double position;
    private static double maximExtendoPosition=1300;

    DigitalChannel isIn;
    double velocity=0;

    public Extendo()
    {
        Differential.init();
        encoder=new Encoder(Hardware.meh0 , encoderReversed);
        controller=new PIDController(kp , ki , kd);
        state=State.OUT;
        isIn=Hardware.extendoBeamBreak;
    }

    public void setVelocity(double velocity)
    {
        if(Math.abs(velocity)<0.05){this.velocity=0;return;}
        this.velocity=velocity;
        state=State.OUT;
    }
    public void setIn()
    {
        if(state==State.GOING_IN || state==State.IN)return;
        state=State.GOING_IN;
        Differential.extendoPower=goingInPower;
    }

    public void setTargetPosition(double position)
    {
        state=State.GO_TO_POSITION;
        targetPosition=position;
    }

    public boolean inPosition()
    {
        return (state==State.IN || Math.abs(encoder.getPosition()-targetPosition)<40 || encoder.getPosition()>1000) && state!=State.GOING_IN;
    }

    private void updateHardware()
    {
        switch (state)
        {
            case IN:
                nr=0;
                Differential.extendoPower=inPower;
                break;
            case OUT:
                nr=0;
                if(encoder.getPosition()<maximExtendoPosition)
                {Differential.extendoPower=velocity;}
                else {Differential.extendoPower=(Math.min(velocity , 0.1));}
                if( Math.abs(velocity)<0.005 && encoder.getPosition()<100){
                    state=state.nextState;
                    Differential.extendoPower=goingInPower;
                }
                break;
            case GOING_IN:
                Differential.extendoPower=goingInPower;
                Differential.update();
                if(Math.abs(encoder.getVelocity())<100)nr++;
                if((Math.abs(encoder.getVelocity())<100 && nr>1)){
                    {state=state.nextState;
                        encoder.resetPosition();}}
                break;
            case GO_TO_POSITION:

                double power=controller.calculate(targetPosition , encoder.getPosition());
                Differential.extendoPower=power;
                break;
        }
    }

    private void updateCoefficient()
    {
        controller.kp=kp;
        controller.ki=ki;
        controller.kd=kd;
    }



    public void update()
    {
        position=encoder.getPosition();
        updateCoefficient();
        updateHardware();
        Differential.update();
    }









}