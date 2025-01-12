package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;
import org.opencv.features2d.MSER;

@Config
public class Ramp extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double upPosition=0.37 , downPosition=0.57;

    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;

    public Ramp()
    {
        moduleName="RAMP";
        setServos(
                new BetterServo("Servo" , Hardware.seh2 , BetterServo.RunMode.PROFILE , upPosition, rightServoReversed)
        );        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();

        setStates();
        atStart();

    }

    @Override
    public void setStates()  {
        states.addState("up" , upPosition);
        states.addState("down" , downPosition);

        states.addState("goUp" , states.get("up"), upPosition);
        states.addState("goDown",  states.get("down"), downPosition);


    }

    @Override
    public void updateStatesPosition(){
        states.get("up").updatePositions(upPosition);
        states.get("down").updatePositions(downPosition);

        states.get("goUp").updatePositions(upPosition);
        states.get("goDown").updatePositions(downPosition);
    }

    @Override
    public void atStart()  {
        state=states.get("goUp");
    }

}