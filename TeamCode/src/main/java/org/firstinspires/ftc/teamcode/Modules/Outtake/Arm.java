package org.firstinspires.ftc.teamcode.Modules.Outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.IServoModule;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Arm extends IServoModule {

    public static boolean leftServoReversed=true , rightServoReversed=false;

    public static double defaultLeft=0.24 , defaultRight=0.53;

    public static double depositLeft=0.23 , depositRight=0.43;

    public static double takeSpecimenBackLeft=0.675 , takeSpecimenBackRight=0.5;

    public static double takeSpecimenFrontLeft=0.5 , takeSpecimenFrontRight=0.5;

    public static double withElementLeft=0.39 , withElementRight=0.65;

    public static double releaseSampleFrontLeft=0.5 , releaseSampleFrontRight=0.5;

    public static double releaseSampleBackLeft=0.5 , releaseSampleBackRight=0.5;

    public static double lowSampleFrontLeft=0.4 , lowSampleFrontRight=0.4;

    public static double lowSampleBackLeft=0.6 , lowSampleBackRight=0.6;

    public static double highSampleFrontLeft=0.4 , highSampleFrontRight=0.5;

    public static double putHighSampleBackLeft=0.65 , putHighSampleBackRight=0.5;

    public static double lowSpecimenFrontLeft=0.5 , lowSpecimenFrontRight=0.5;

    public static double lowSpecimenBackLeft=0.5 , lowSpecimenBackRight=0.5;

    public static double highSpecimenFrontLeft=0.4 , highSpecimenFrontRight=0.565;

    public static double highSpecimenBackLeft=0.5 , highSpecimenBackRight=0.5;

    public static double beforeTakeSpecimenLeft=0.55 , beforeTakeSpecimenRight=0.5;

    public static double MaxVelocoty=15 , Acceleration=12  , Deceleration=12;

    public static double neutralLeft=0.5 , neutralRight=0.5;

    public Arm()
    {
        moduleName="ARM";

        setServos(new BetterServo("ServoLeft" , Hardware.seh4 , BetterServo.RunMode.PROFILE , defaultLeft , leftServoReversed),
                new BetterServo("ServoRight" , Hardware.seh3 , BetterServo.RunMode.PROFILE ,  defaultRight , rightServoReversed)
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


        states.addState("default" , defaultLeft , defaultRight);
        states.addState("goDefault" , states.get("default") , defaultLeft , defaultRight);

        states.addState("deposit" , depositLeft , depositRight);
        states.addState("goDeposit" , states.get("deposit") , depositLeft , depositRight);

        states.addState("takeSpecimenBack" , takeSpecimenBackLeft , takeSpecimenBackRight);
        states.addState("goTakeSpecimenBack" , states.get("takeSpecimenBack") , takeSpecimenBackLeft , takeSpecimenBackRight);
        states.addState("takeSpecimenFront" , takeSpecimenFrontLeft , takeSpecimenFrontRight);

        states.addState("withElement" , withElementLeft, withElementRight);
        states.addState("goWithElement" , states.get("withElement") , withElementLeft, withElementRight);

        states.addState("releaseSampleFront" , releaseSampleFrontLeft, releaseSampleFrontRight);
        states.addState("goReleaseSampleFront" , states.get("releaseSampleFront") , releaseSampleFrontLeft, releaseSampleFrontRight);

        states.addState("releaseSampleBack" , releaseSampleBackLeft , releaseSampleBackRight);
        states.addState("goReleaseSampleBack" , states.get("releaseSampleBack") , releaseSampleBackLeft , releaseSampleBackRight);

        states.addState("lowSampleFront" , lowSampleFrontLeft , lowSampleFrontRight);
        states.addState("goLowSampleFront" , states.get("lowSampleFront") , lowSampleFrontLeft , lowSampleFrontRight);

        states.addState("lowSampleBack", lowSampleBackLeft , lowSampleBackRight);
        states.addState("goLowSampleBack" , states.get("lowSampleBack") , lowSampleBackLeft , lowSampleBackRight);

        states.addState("highSampleFront" , highSampleFrontLeft , highSampleFrontRight);
        states.addState("goHighSampleFront" , states.get("highSampleFront") , highSampleFrontLeft , highSampleFrontRight);

        states.addState("highSampleBack" , putHighSampleBackLeft , putHighSampleBackRight);
        states.addState("goHighSampleBack" , states.get("highSampleBack") , putHighSampleBackLeft , putHighSampleBackRight);

        states.addState("lowSpecimenFront" , lowSpecimenFrontLeft , lowSpecimenFrontRight);
        states.addState("goLowSpecimenFront" , states.get("lowSpecimenFront") , lowSpecimenFrontLeft , lowSpecimenFrontRight);

        states.addState("lowSpecimenBack" , lowSpecimenBackLeft , lowSpecimenBackRight);
        states.addState("goLowSpecimenBack" , states.get("lowSpecimenBack") , lowSpecimenBackLeft , lowSpecimenBackRight);

        states.addState("highSpecimenFront" , highSpecimenFrontLeft , highSpecimenFrontRight);
        states.addState("goHighSpecimenFront" , states.get("highSpecimenFront") , highSpecimenFrontLeft , highSpecimenFrontRight);

        states.addState("highSpecimenBack" , highSpecimenBackLeft , highSpecimenBackRight);
        states.addState("goHighSpecimenBack" , states.get("highSpecimenBack") , highSpecimenBackLeft , highSpecimenBackRight);

        states.addState("goTakeSpecimenFront" , states.get("takeSpecimenFront") , takeSpecimenFrontLeft , takeSpecimenFrontRight);

        states.addState("neutral" , neutralLeft , neutralRight);
        states.addState("goNeutral" , states.get("neutral"), neutralLeft , neutralRight);

        states.addState("beforeTakeSpecimen" , beforeTakeSpecimenLeft , beforeTakeSpecimenRight);
        states.addState("goBeforeTakeSpecimen" ,  states.get("beforeTakeSpecimen"),beforeTakeSpecimenLeft , beforeTakeSpecimenRight);

    }

    @Override
    public void updateStatesPosition(){


        states.get("neutral").updatePositions(neutralLeft , neutralRight);
        states.get("goNeutral").updatePositions(neutralLeft , neutralRight);

        states.get("default").updatePositions(defaultLeft , defaultRight);
        states.get("goDefault").updatePositions(defaultLeft , defaultRight);

        states.get("deposit").updatePositions(depositLeft , depositRight);
        states.get("goDeposit").updatePositions(depositLeft , depositRight);

        states.get("takeSpecimenBack").updatePositions(takeSpecimenBackLeft , takeSpecimenBackRight);
        states.get("goTakeSpecimenBack").updatePositions(takeSpecimenBackLeft , takeSpecimenBackRight);

        states.get("takeSpecimenFront").updatePositions(takeSpecimenFrontLeft , takeSpecimenFrontRight);
        states.get("goTakeSpecimenFront").updatePositions(takeSpecimenFrontLeft , takeSpecimenFrontRight);

        states.get("withElement").updatePositions(withElementLeft , withElementRight);
        states.get("goWithElement").updatePositions(withElementLeft , withElementRight);

        states.get("releaseSampleFront").updatePositions(releaseSampleFrontLeft , releaseSampleFrontRight);
        states.get("goReleaseSampleFront").updatePositions(releaseSampleFrontLeft , releaseSampleFrontRight);

        states.get("releaseSampleBack").updatePositions(releaseSampleBackLeft , releaseSampleBackRight);
        states.get("goReleaseSampleBack").updatePositions(releaseSampleBackLeft , releaseSampleBackRight);

        states.get("lowSampleFront").updatePositions(lowSampleFrontLeft , lowSampleFrontRight);
        states.get("goLowSampleFront").updatePositions(lowSampleFrontLeft , lowSampleFrontRight);

        states.get("lowSampleBack").updatePositions(lowSampleBackLeft , lowSampleBackRight);
        states.get("goLowSampleBack").updatePositions(lowSampleBackLeft , lowSampleBackRight);

        states.get("highSampleFront").updatePositions(highSampleFrontLeft , highSampleFrontRight);
        states.get("goHighSampleFront").updatePositions(highSampleFrontLeft , highSampleFrontRight);

        states.get("highSampleBack").updatePositions(putHighSampleBackLeft , putHighSampleBackRight);
        states.get("goHighSampleBack").updatePositions(putHighSampleBackLeft , putHighSampleBackRight);

        states.get("lowSpecimenFront").updatePositions(lowSpecimenFrontLeft , lowSpecimenFrontRight);
        states.get("goLowSpecimenFront").updatePositions(lowSpecimenFrontLeft , lowSpecimenFrontRight);

        states.get("lowSpecimenBack").updatePositions(lowSpecimenBackLeft , lowSpecimenBackRight);
        states.get("goLowSpecimenBack").updatePositions(lowSpecimenBackLeft , lowSpecimenBackRight);

        states.get("highSpecimenFront").updatePositions(highSpecimenFrontLeft , highSpecimenFrontRight);
        states.get("goHighSpecimenFront").updatePositions(highSpecimenFrontLeft , highSpecimenFrontRight);

        states.get("highSpecimenBack").updatePositions(highSpecimenBackLeft , highSpecimenBackRight);
        states.get("goHighSpecimenBack").updatePositions(highSpecimenBackLeft , highSpecimenBackRight);

        states.get("beforeTakeSpecimen").updatePositions(beforeTakeSpecimenLeft , beforeTakeSpecimenRight);
        states.get("goBeforeTakeSpecimen").updatePositions(beforeTakeSpecimenLeft , beforeTakeSpecimenRight);

    }

    @Override
    public void atStart()  {
        state=states.get("default");
    }
}