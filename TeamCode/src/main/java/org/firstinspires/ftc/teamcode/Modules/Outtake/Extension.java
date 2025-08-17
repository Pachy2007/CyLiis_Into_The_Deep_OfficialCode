package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Extension extends IServoModule {


    public static double extend=0.19;

    public static double retrect=0.735;

    public static double deposit=0.355;

    public static double takeSample=0.355;

    public static double takeSpecimen=0.605;

    public static double takeSpecimenSpecial=0.4;

    State initState;

    public static boolean reversed=false;

    public static double MaxVelocoty=18 , Acceleration=14  , Deceleration=14;

    public Extension()
    {
        moduleName="EXTENSION";
        setStates();
        initState=states.get("retrect");
        setServos(
                new BetterServo("Servo" , Hardware.ssh4 , BetterServo.RunMode.PROFILE , retrect , reversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();
        atStart();
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

        states.addState("takeSpecimenSpecial" , takeSpecimenSpecial);
    }

    @Override
    public void updateStatesPosition() {

        states.get("takeSpecimenSpecial").updatePositions(takeSpecimenSpecial);

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
