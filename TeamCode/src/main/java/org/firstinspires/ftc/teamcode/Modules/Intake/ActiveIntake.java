package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class ActiveIntake {

    public static double repausPower=0 , intakePower=1 , reversePower=-0.75;

    public enum State{
        REPAUS(repausPower) , INTAKE(intakePower) , REVERSE(reversePower);

        double power;
        State(double power)
        {
            this.power=power;
        }
    }
    State state;

    public static boolean reversed=true;
    BetterMotor motor;

    public ActiveIntake()
    {
        motor= new BetterMotor(Hardware.meh3 , BetterMotor.RunMode.RUN , reversed);
        state=State.REPAUS;
        motor.motor.setCurrentAlert(7.5 , CurrentUnit.AMPS);
    }


    public void setMode(State state)
    {
        this.state=state;
        motor.setPower(state.power);
    }
}