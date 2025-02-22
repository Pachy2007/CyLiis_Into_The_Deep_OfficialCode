package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Claw extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double closeSample=0.44 ,  closeSpecimen=0.46 , openPosition=0.35, takeSpecimenPosition=0.23 , scoring=0.23;
    State initState;
    public Claw()
    {
        moduleName="CLAW";
        setServos(
                new BetterServo("Servo" , Hardware.sch1 , BetterServo.RunMode.Time ,  openPosition , rightServoReversed , 0.15)
        );
        setStates();
        initState=states.get("goOpen");
        atStart();

    }

    public Claw(String string)
    {
        moduleName="CLAW";
        setStates();
        initState=states.get(string);
        setServos(
                new BetterServo("Servo" , Hardware.sch1 , BetterServo.RunMode.Time ,  initState.getPosition(0) , rightServoReversed , 0.15)
        );

        atStart();
    }

    @Override
    public void setStates()  {
        states.addState("close" , closeSample);
        states.addState("open"  ,openPosition);
        states.addState("takeSpecimen" , takeSpecimenPosition);
        states.addState("closeSpecimen" , closeSpecimen);

        states.addState("scoring" , scoring);

        states.addState("goOpen" , states.get("open"), openPosition);
        states.addState("goClose",  states.get("close"), closeSample);
        states.addState("goTakeSpecimen" , states.get("takeSpecimen") , takeSpecimenPosition);
        states.addState("goCloseSpecimen" , states.get("closeSpecimen") , closeSpecimen);

        states.addState("goScoring" , states.get("scoring"), scoring);
    }

    @Override
    public void updateStatesPosition(){

        states.get("scoring").updatePositions(scoring);
        states.get("goScoring").updatePositions(scoring);

        states.get("close").updatePositions(closeSample);
        states.get("open").updatePositions(openPosition);
        states.get("takeSpecimen").updatePositions(takeSpecimenPosition);
        states.get("closeSpecimen").updatePositions(closeSpecimen);

        states.get("goOpen").updatePositions(openPosition);
        states.get("goClose").updatePositions(closeSample);
        states.get("goTakeSpecimen").updatePositions(takeSpecimenPosition);
        states.get("goCloseSpecimen").updatePositions(closeSpecimen);
    }

    @Override
    public void atStart()  {
        state=initState;
    }
}