package org.firstinspires.ftc.teamcode.Modules.Others;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.opencv.core.Mat;

public class Differential {

    public static double extendoPower=0;
    public static double liftPower=0;

    public static double power1 , power2;

    public static BetterMotor motor1 , motor2;
    public static void init()
    {
        motor1=new BetterMotor(Hardware.mch3 , BetterMotor.RunMode.RUN,false);
        motor2=new BetterMotor(Hardware.mch2 , BetterMotor.RunMode.RUN,false);
    }




    public static void update()
    {



        liftPower=Math.max(-1 , Math.min(1 , liftPower));
        extendoPower=Math.max(-1 , Math.min(1, extendoPower));

        if(liftPower!=0 && extendoPower==-0.02)extendoPower=0;


         power1=extendoPower-liftPower;
         power2=liftPower+extendoPower;

        double denominator= Math.max(1 , Math.max(Math.abs(power1), Math.abs(power2)));

        power1/=denominator;
        power2/=denominator;

        motor1.setPower(power1);
        motor2.setPower(power2);
    }

}
