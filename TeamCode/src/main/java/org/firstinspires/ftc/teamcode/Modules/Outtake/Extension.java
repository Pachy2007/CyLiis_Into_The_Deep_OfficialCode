package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Extension extends IServoModule {


    public static double extend=0.2;

    public static double retrect=0.75;

    public static double deposit=0.35;

    public static double takeSample=0.38;

    public static double takeSpecimen=0.65;

    State initState;

    public static boolean reversed=false;

    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;

    public Extension()
    {
        moduleName="EXTENSION";
        setStates();
        initState=states.get("retrect");
        setServos(
                new BetterServo("Servo" , Hardware.ssh0 , BetterServo.RunMode.PROFILE , retrect , reversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();
        atStart();
        update();
    }

    @Override
    public void setStates() {
        states.addState("extend" , extend);
        states.addState("goExtend" , states.get("extend") , extend);

        states.addState("retrect" , retrect);
        states.addState("goRetrect" , states.get("retrect") , retrect);

        states.addState("deposit" , deposit);
        states.addState("goDeposit" , states.get("deposit") , deposit);

        states.addState("takeSpecimen" , takeSpecimen);
        states.addState("goTakeSpecimen" , states.get("takeSpecimen") , takeSpecimen);

        states.addState("takeSample" , takeSample);
        states.addState("goTakeSample" , states.get("takeSample") , takeSample);
    }

    @Override
    public void updateStatesPosition() {
        states.get("extend").updatePositions(extend);
        states.get("goExtend").updatePositions(extend);

        states.get("retrect").updatePositions(retrect);
        states.get("goRetrect").updatePositions(retrect);

        states.get("deposit").updatePositions(deposit);
        states.get("goDeposit").updatePositions(deposit);

        states.get("takeSpecimen").updatePositions(takeSpecimen);
        states.get("goTakeSpecimen").updatePositions(takeSpecimen);

        states.get("takeSample").updatePositions(takeSample);
        states.get("goTakeSample").updatePositions(takeSample);
    }

    @Override
    public void atStart() {
        state=initState;
    }
}
