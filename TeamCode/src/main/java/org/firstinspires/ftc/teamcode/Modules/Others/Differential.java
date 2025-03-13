package org.firstinspires.ftc.teamcode.Modules.Others;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.opencv.core.Mat;

public class Differential {

    public static double extendoPower=0;
    public static double liftPower=0;

    public static double power1 , power2;

    public static BetterMotor motor1 , motor2, boostLift;
    public static void init()
    {
        motor1=new BetterMotor(Hardware.meh2 , BetterMotor.RunMode.RUN,false);
        motor2=new BetterMotor(Hardware.mch3 , BetterMotor.RunMode.RUN,false);
        boostLift=new BetterMotor(Hardware.mch2 , BetterMotor.RunMode.RUN , true);
        extendoPower=0;
        liftPower=0;

        motor1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        boostLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        update();
    }




    public static void update()
    {



        liftPower=Math.max(-1 , Math.min(1 , liftPower));
        extendoPower=Math.max(-1 , Math.min(1, extendoPower));

         power1=extendoPower-liftPower;
         power2=liftPower+extendoPower;

        double denominator= Math.max(1 , Math.max(Math.abs(power1), Math.abs(power2)));

        power1/=denominator;
        power2/=denominator;

        double powerForBoost=(power2-power1)/2;

        motor1.setPower(power1);
        motor2.setPower(power2);
        boostLift.setPower(powerForBoost);
    }

}
