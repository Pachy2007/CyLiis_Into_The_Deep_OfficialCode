package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Arm extends IServoModule {

    public static boolean leftServoReversed=true , rightServoReversed=false ;


    public static double deposit=0.03;

    public static double takeSpecimen=0.905;

    public static double withElementSample=0.045 , withElementSpecimen=0.27;

    public static double releaseSample=0.2;

    public static double lowSample=0.62;

    public static double putHighSample=0.58;

    public static double lowSpecimen=0.27;

    public static double neutralSpecimen=0.55 , neutralSample=0.55;

    public static double highSpecimen=0.255;

    public static double scoreSample=0.7;


    public static double MaxVelocoty=20 , Acceleration=32  , Deceleration=32;

    State initState;

    public Arm()
    {
        moduleName="ARM";

        setStates();
        initState=states.get("deposit");
        setServos(new BetterServo("ServoLeft" , Hardware.ssh2 , BetterServo.RunMode.PROFILE , deposit , leftServoReversed),
                new BetterServo("ServoRight" , Hardware.ssh1 , BetterServo.RunMode.PROFILE ,  deposit , rightServoReversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;

        setProfileCoefficients();
        atStart();

    }

    public Arm(String string)
    {
        moduleName="ARM";

        setStates();
        initState=states.get(string);
        setServos(new BetterServo("ServoLeft" , Hardware.ssh2 , BetterServo.RunMode.PROFILE , deposit , leftServoReversed),
                new BetterServo("ServoRight" , Hardware.ssh1 , BetterServo.RunMode.PROFILE ,  deposit , rightServoReversed)
        );
        this.maxVelocity=MaxVelocoty;
        this.acceleration=Acceleration;
        this.deceleration=Deceleration;

        setProfileCoefficients();
        atStart();

    }

    @Override
    public void setStates()  {

        states.addState("deposit" , deposit , deposit);
        states.addState("goDeposit" , states.get("deposit") , deposit , deposit);

        states.addState("withElementSample" , withElementSample, withElementSample);
        states.addState("goWithElementSample" , states.get("withElementSample") , withElementSample, withElementSample);

        states.addState("withElementSpecimen" , withElementSpecimen, withElementSpecimen);
        states.addState("goWithElementSpecimen" , states.get("withElementSpecimen") , withElementSpecimen, withElementSpecimen);

        states.addState("releaseSampleBack" , releaseSample , releaseSample);
        states.addState("goReleaseSampleBack" , states.get("releaseSampleBack") , releaseSample , releaseSample);

        states.addState("lowSample", lowSample , lowSample);
        states.addState("goLowSample" , states.get("lowSample") , lowSample , lowSample);

        states.addState("highSample" , putHighSample , putHighSample);
        states.addState("goHighSample" , states.get("highSample") , putHighSample , putHighSample);

        states.addState("lowSpecimen" , lowSpecimen , lowSpecimen);
        states.addState("goLowSpecimen" , states.get("lowSpecimen") , lowSpecimen , lowSpecimen);

        states.addState("highSpecimen" , highSpecimen , highSpecimen);
        states.addState("goHighSpecimen" , states.get("highSpecimen") , highSpecimen , highSpecimen);

        states.addState("takeSpecimen" , takeSpecimen , takeSpecimen);
        states.addState("goTakeSpecimen" , states.get("takeSpecimen") , takeSpecimen , takeSpecimen);

        states.addState("neutralSpecimen" , neutralSpecimen , neutralSpecimen);
        states.addState("goNeutralSpecimen" , states.get("neutralSpecimen"), neutralSpecimen , neutralSpecimen);

        states.addState("neutralSample" , neutralSample , neutralSample);
        states.addState("goNeutralSample" , states.get("neutralSample"), neutralSample , neutralSample);

        states.addState("scoreSample" , scoreSample , scoreSample);
        states.addState("goScoreSample", states.get("scoreSample"), scoreSample , scoreSample);



    }

    @Override
    public void updateStatesPosition(){

        states.get("neutralSample").updatePositions(neutralSample , neutralSample);
        states.get("goNeutralSample").updatePositions(neutralSample , neutralSample);

        states.get("neutralSpecimen").updatePositions(neutralSpecimen , neutralSpecimen);
        states.get("goNeutralSpecimen").updatePositions(neutralSpecimen , neutralSpecimen);

        states.get("deposit").updatePositions(deposit , deposit);
        states.get("goDeposit").updatePositions(deposit , deposit);

        states.get("takeSpecimen").updatePositions(takeSpecimen , takeSpecimen);
        states.get("goTakeSpecimen").updatePositions(takeSpecimen , takeSpecimen);

        states.get("withElementSample").updatePositions(withElementSample , withElementSample);
        states.get("goWithElementSample").updatePositions(withElementSample , withElementSample);

        states.get("withElementSpecimen").updatePositions(withElementSpecimen , withElementSpecimen);
        states.get("goWithElementSpecimen").updatePositions(withElementSpecimen , withElementSpecimen);

        states.get("releaseSampleBack").updatePositions(releaseSample , releaseSample);
        states.get("goReleaseSampleBack").updatePositions(releaseSample , releaseSample);

        states.get("lowSample").updatePositions(lowSample, lowSample);
        states.get("goLowSample").updatePositions(lowSample , lowSample);

        states.get("highSample").updatePositions(putHighSample , putHighSample);
        states.get("goHighSample").updatePositions(putHighSample , putHighSample);

        states.get("lowSpecimen").updatePositions(lowSpecimen , lowSpecimen);
        states.get("goLowSpecimen").updatePositions(lowSpecimen , lowSpecimen);

        states.get("highSpecimen").updatePositions(highSpecimen , highSpecimen);
        states.get("goHighSpecimen").updatePositions(highSpecimen , highSpecimen);

        states.get("scoreSample").updatePositions(scoreSample , scoreSample);
        states.get("goScoreSample").updatePositions(scoreSample , scoreSample);

    }

    @Override
    public void atStart()  {
        state=initState;
    }
}