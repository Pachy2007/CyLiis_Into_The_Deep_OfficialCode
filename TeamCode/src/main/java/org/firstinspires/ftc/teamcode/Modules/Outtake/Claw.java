package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Claw extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double closeSample=0.82 ,  closeSpecimen=1 , openPosition=0.65 , takeSpecimenPosition=0.4;

    public Claw()
    {
        moduleName="CLAW";
        setServos(
                new BetterServo("Servo" , Hardware.seh5 , BetterServo.RunMode.Time ,  openPosition , rightServoReversed , 0.4)
        );
        setStates();
        atStart();

    }

    @Override
    public void setStates()  {
        states.addState("close" , closeSpecimen);
        states.addState("open"  ,openPosition);
        states.addState("takeSpecimen" , takeSpecimenPosition);
        states.addState("closeSpecimen" , closeSpecimen);

        states.addState("goOpen" , states.get("open"), openPosition);
        states.addState("goClose",  states.get("close"), closeSpecimen);
        states.addState("goTakeSpecimen" , states.get("takeSpecimen") , takeSpecimenPosition);
        states.addState("goCloseSpecimen" , states.get("closeSpecimen") , closeSpecimen);
    }

    @Override
    public void updateStatesPosition(){
        states.get("close").updatePositions(closeSpecimen);
        states.get("open").updatePositions(openPosition);
        states.get("takeSpecimen").updatePositions(takeSpecimenPosition);
        states.get("closeSpecimen").updatePositions(closeSpecimen);

        states.get("goOpen").updatePositions(openPosition);
        states.get("goClose").updatePositions(closeSpecimen);
        states.get("goTakeSpecimen").updatePositions(takeSpecimenPosition);
        states.get("goCloseSpecimen").updatePositions(closeSpecimen);
    }

    @Override
    public void atStart()  {
        state=states.get("close");
    }
}