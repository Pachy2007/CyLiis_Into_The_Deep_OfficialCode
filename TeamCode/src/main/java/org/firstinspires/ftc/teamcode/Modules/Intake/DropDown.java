package org.firstinspires.ftc.teamcode.Modules.Intake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;
import org.opencv.features2d.MSER;

@Config
public class DropDown extends IServoModule {

    public static boolean rightServoReversed=false;

    public static double takeElementPosition=0.1, rampUpPosition=0.4, rampDownPosition=0.4 , jamPosition=0.7;

    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;

    public DropDown()
    {
        moduleName="DropDOWN";
        setServos(
                new BetterServo("Servo" , Hardware.seh1 , BetterServo.RunMode.PROFILE ,  rampUpPosition , rightServoReversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;
        setProfileCoefficients();

        setStates();
        atStart();

    }

    @Override
    public void setStates()  {
        states.addState("takeElement" , takeElementPosition);
        states.addState("jam" , jamPosition);
        states.addState("rampUp" , rampUpPosition);
        states.addState("rampDown" , rampDownPosition);

        states.addState("goTakeElement" , states.get("takeElement") , takeElementPosition);
        states.addState("goRampUp" , states.get("rampUp") , rampUpPosition);
        states.addState("goRampDown" , states.get("rampDown") , rampDownPosition);
        states.addState("goJam" , states.get("jam") , jamPosition);



    }

    @Override
    public void updateStatesPosition(){
        states.get("takeElement").updatePositions(takeElementPosition);
        states.get("goTakeElement").updatePositions(takeElementPosition);

        states.get("rampUp").updatePositions(rampUpPosition);
        states.get("goRampUp").updatePositions(rampUpPosition);

        states.get("rampDown").updatePositions(rampDownPosition);
        states.get("goRampDown").updatePositions(rampDownPosition);

        states.get("jam").updatePositions(jamPosition);
        states.get("goJam").updatePositions(jamPosition);
    }

    @Override
    public void atStart()  {
        state=states.get("goRampUp");
    }

}