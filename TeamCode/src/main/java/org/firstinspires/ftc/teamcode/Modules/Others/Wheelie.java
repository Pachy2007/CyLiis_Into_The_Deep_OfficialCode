package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.network.ApChannel;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.State;

public class Wheelie {

    enum State{
        GOING_DOWN , RELEASE
    }
    State state;

    public boolean goDown=false;
    public boolean up=false;

    public double goDownPower=-1 , keepPositionPower=-0.07;
    Servo servoLeft , servoRight;

    public Wheelie()
    {
        servoLeft= Hardware.ssh4;
        servoRight=Hardware.ssh5;
        state=State.RELEASE;
    }

    public void keepUp()
    {

        state=State.RELEASE;
    }
    public void goDown()
    {

        state=State.GOING_DOWN;
    }

    private void updateHardware()
    {

        switch (state)
        {
            case RELEASE:
                servoLeft.setPosition(0.5);
                servoRight.setPosition(0.5);
                break;
            case GOING_DOWN:
                if(goDown)
                {
                    if(!up)
                    {servoRight.setPosition(0);
                    servoLeft.setPosition(0);}
                    else
                    {servoRight.setPosition(1);
                    servoLeft.setPosition(1);}
                }
                else{
                    servoRight.setPosition(0.5);
                    servoLeft.setPosition(0.5);
                }
                break;

        }
    }

    public void update()
    {
        updateHardware();
    }


}
