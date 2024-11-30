package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.opencv.core.Mat;

@Config
public class PidAngle {

    public static double kp , ki , kd;
    public static PIDController controller;

    public static void init()
    {
        controller=new PIDController(kp , ki ,kd);
    }


    public static double calculate(double a1 , double a2)
    {
        controller.kp=kp;
        controller.ki=ki;
        controller.kd=kd;
        double b1=(2*Math.PI+a1)- Math.floor((2*Math.PI+a1)/(Math.PI*2))*Math.PI*2;
        double b2=(2*Math.PI+a2)- Math.floor((2*Math.PI+a2)/(Math.PI*2))*Math.PI*2;

        double z=b2-b1;

        if(Math.abs(z)>=Math.PI/2)z=Math.signum(-z)*(Math.PI*2-Math.abs(z));

        double power= controller.calculate(z , 0);

        return power;
    }
}