package org.firstinspires.ftc.teamcode.Modules.Others;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class PTO extends IServoModule {

    public static boolean reversed=false;

    public static double climbPosition=0;
    public static double normalPosition=0.34;

    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;

    public PTO()
    {
        moduleName="PTO";
        setServos(
                new BetterServo("PTO" , Hardware.ssh3  , BetterServo.RunMode.PROFILE , normalPosition , reversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();
        setStates();
        atStart();
    }
    @Override
    public void setStates() {

        states.addState("normal" , normalPosition);
        states.addState("goNormal" , states.get("normal") , normalPosition);

        states.addState("climb" , climbPosition);
        states.addState("goClimb" , states.get("climb") , climbPosition);
    }

    @Override
    public void updateStatesPosition() {

    }

    @Override
    public void atStart() {
        state=states.get("normal");
    }
}
