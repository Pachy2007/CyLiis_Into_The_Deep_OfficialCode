package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Claw extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double closeSample=0.63 ,  closeSpecimen=0.7 , openPosition=0.41 , takeSpecimenPosition=0.1;
    State initState;
    public Claw()
    {
        moduleName="CLAW";
        setServos(
                new BetterServo("Servo" , Hardware.sch5 , BetterServo.RunMode.Time ,  openPosition , rightServoReversed , 0.15)
        );
        setStates();
        initState=states.get("goOpen");
        atStart();

    }

    public Claw(String string)
    {
        setStates();
        initState=states.get(string);
        moduleName="CLAW";
        setServos(
                new BetterServo("Servo" , Hardware.sch5 , BetterServo.RunMode.Time ,  initState.getPosition(0) , rightServoReversed , 0.15)
        );

        atStart();
    }

    @Override
    public void setStates()  {
        states.addState("close" , closeSample);
        states.addState("open"  ,openPosition);
        states.addState("takeSpecimen" , takeSpecimenPosition);
        states.addState("closeSpecimen" , closeSpecimen);

        states.addState("goOpen" , states.get("open"), openPosition);
        states.addState("goClose",  states.get("close"), closeSample);
        states.addState("goTakeSpecimen" , states.get("takeSpecimen") , takeSpecimenPosition);
        states.addState("goCloseSpecimen" , states.get("closeSpecimen") , closeSpecimen);
    }

    @Override
    public void updateStatesPosition(){
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