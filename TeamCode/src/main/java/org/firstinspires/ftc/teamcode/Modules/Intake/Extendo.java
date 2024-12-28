package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
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
    public BetterMotor motor;
    public static boolean motor1Reversed=true;
    public static double kp=0.01 , ki , kd;
    public static double targetPosition;

    private static double inPower=-0.1 , goingInPower=-1;

    private static double maximExtendoPosition=1000;

    double velocity;

    public Extendo()
    {
        motor=new BetterMotor(Hardware.meh2 , BetterMotor.RunMode.RUN , motor1Reversed , Hardware.mch2 , false);

        motor.setPidCoefficients(kp , ki , kd);

        state=State.OUT;

    }

    public void setVelocity(double velocity)
    {
        if(Math.abs(velocity)<0.05){this.velocity=0;return;}
        this.velocity=velocity;
        state=State.OUT;
        motor.setRunMode(BetterMotor.RunMode.RUN);
    }
    public void setIn()
    {
        motor.setRunMode(BetterMotor.RunMode.RUN);
        if(state==State.GOING_IN || state==State.IN)return;
        state=State.GOING_IN;
        motor.setPower(goingInPower);
    }

    public void setTargetPosition(double position)
    {
        state=State.GO_TO_POSITION;
        motor.setRunMode(BetterMotor.RunMode.PID);
        motor.setPosition(targetPosition);
        targetPosition=position;
    }

    public boolean inPosition()
    {
        return state==State.IN || state!=State.GOING_IN || Math.abs(motor.getPosition()-targetPosition)<40;
    }

    private void updateHardware()
    {
        switch (state)
        {
            case IN:
                nr=0;
                motor.setPower(inPower);
                break;
            case OUT:
                nr=0;
                if(motor.getPosition()<maximExtendoPosition)
                {motor.setPower(velocity);}
                else {motor.setPower(Math.min(velocity , 0.1));}
                if( Math.abs(velocity)<0.005 && motor.getPosition()<100){
                    state=state.nextState;
                    motor.setPower(goingInPower);
                }
                break;
            case GOING_IN:
                motor.setPower(goingInPower);
                if(Math.abs(motor.getVelocity())<0.0001){
                    nr++;
                    if(nr>=2)
                    {state=state.nextState;
                        motor.resetPosition();}}
                break;
            case GO_TO_POSITION:
                motor.setPosition(targetPosition);
                break;
        }
    }

    private void updateCoefficient()
    {
        motor.setPidCoefficients(kp , ki , kd);
    }



    public void update()
    {
        updateCoefficient();
        updateHardware();
        motor.update();
    }









}