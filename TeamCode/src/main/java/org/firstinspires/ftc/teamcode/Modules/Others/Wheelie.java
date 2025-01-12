package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.CRServo;

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
    CRServo servoLeft , servoRight;

    public Wheelie()
    {
        servoLeft= Hardware.sch2;
        servoRight=Hardware.sch3;
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
                servoLeft.setPower(0);
                servoRight.setPower(0);
                break;
            case GOING_DOWN:
                if(goDown)
                {
                    if(!up)
                    {servoRight.setPower(goDownPower);
                    servoLeft.setPower(goDownPower);}
                    else
                    {servoRight.setPower(-goDownPower);
                    servoLeft.setPower(-goDownPower);}
                }
                else{
                    servoRight.setPower(keepPositionPower);
                    servoLeft.setPower(keepPositionPower);
                }
                break;

        }
    }

    public void update()
    {
        updateHardware();
    }


}
