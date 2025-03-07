package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Modules.Outtake.Extension;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;
import org.opencv.features2d.MSER;

@Config
public class Ramp extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double upPosition=0.7 , downPosition=0.515;

    public static double MaxVelocoty=5 , Acceleration=20  , Deceleration=20;


    public static double rampPositionIn=0.95, rampPositionOut=0.88;
    public static double extendoPositionIn=33 , extendoPositionOut=1250;

    public Ramp()
    {
        moduleName="RAMP";
        setServos(
                new BetterServo("Servo" , Hardware.seh5 , BetterServo.RunMode.PROFILE , upPosition, rightServoReversed)
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

    private void updateRampPosition()
    {
        double x=Extendo.position;
        downPosition=(Extendo.position-extendoPositionIn)*(rampPositionOut-rampPositionIn)/(extendoPositionOut-extendoPositionIn)+rampPositionIn;
    }

    @Override
    public void update() {
        if(state==states.get("goDown") || state==states.get("down"))servos[0].runMode= BetterServo.RunMode.GoToPosition;
        else servos[0].runMode= BetterServo.RunMode.PROFILE;
        updateRampPosition();
        updateStatesPosition();
        updateState();
        updateHardware();
    }
}