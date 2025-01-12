package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;

public class ActiveIntake {

    public static double repausPower=0 , intakePower=1 , reversePowerTeleOp=-0.5 , reversePowerAuto=-1;



    public enum State{
        REPAUS(repausPower) , INTAKE(intakePower) , REVERSE(reversePowerAuto);

        public double power;
        State(double power)
        {
            this.power=power;
        }
    }
    public State state;

    public static boolean reversed=false;
    BetterMotor motor;

    public ActiveIntake()
    {
        motor= new BetterMotor(Hardware.meh2 , BetterMotor.RunMode.RUN , reversed);
        state=State.REPAUS;
        motor.motor.setCurrentAlert(7.5 , CurrentUnit.AMPS);
    }


    public void setMode(State state)
    {

        this.state=state;
        motor.setPower(state.power);

    }
}