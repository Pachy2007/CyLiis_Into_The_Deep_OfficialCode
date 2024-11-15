package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Encoder;

public class Extendo {

    public enum State{
        IN , GOING_IN(IN) , OUT(GOING_IN);

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


    public BetterMotor motor;
    public static boolean motor1Reversed=false;
    public static double kP , kI , kD;

    private static double inPower=-0.1 , goingInPower=-1;

    private static double maximExtendoPosition=700;

    double velocity;

    public Extendo()
    {
        motor=new BetterMotor(Hardware.meh2 , BetterMotor.RunMode.RUN , motor1Reversed , Hardware.mch3);

        motor.setPidCoefficients(kP , kI , kD);

        state=State.OUT;

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
    }


    private void updateHardware()
    {
        switch (state)
        {
            case IN:
                motor.setPower(inPower);
                break;
            case OUT:
                if(motor.getPosition()<maximExtendoPosition)
                {motor.setPower(velocity);}
                else {motor.setPower(0);}
                if( Math.abs(velocity)<0.005 && Math.abs(motor.getPosition())<100){
                    state=state.nextState;
                }
                break;
            case GOING_IN:
                motor.setPower(goingInPower);
                if(Math.abs(motor.getVelocity())<0.000005){state=state.nextState;
                motor.resetPosition();}
                break;
        }
    }

    private void updateCoefficient()
    {
        motor.setPidCoefficients(kP , kI , kD);
    }



    public void update()
    {
        updateCoefficient();
        updateHardware();
    }









}