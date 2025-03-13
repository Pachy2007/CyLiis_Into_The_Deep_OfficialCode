package org.firstinspires.ftc.teamcode.Modules.Others;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

public class Scissor extends IServoModule {

    public static double retrected=0.7;
    public static double deploied=0.3;

    public static boolean reversed=false;

    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;
    public Scissor()
    {
        moduleName="SCISSOR";
        setServos(
                new BetterServo("Servo" , Hardware.seh2 , BetterServo.RunMode.PROFILE , retrected, reversed)
        );        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();
        setStates();
        atStart();
    }


    @Override
    public void setStates() {
        states.addState("retrected" , retrected);
        states.addState("goingRetrected" , states.get("retrected") , retrected);

        states.addState("deploied" , deploied);
        states.addState("goingDeploied" , states.get("deploied") , deploied);
    }

    @Override
    public void updateStatesPosition() {
        states.get("retrected").updatePositions(retrected);
        states.get("goingRetrected").updatePositions(retrected);
        states.get("deploied").updatePositions(deploied);
        states.get("goingDeploied").updatePositions(deploied);
    }

    @Override
    public void atStart() {
        state=states.get("deploied");
    }
}
