package org.firstinspires.ftc.teamcode.Math;

import com.acmerobotics.roadrunner.geometry.Vector2d;

public class PidToPoint {

    public static double kp , ki , kd;

    public static PIDController controller;

    public static void init(){
        controller=new PIDController(kp , ki ,kd);
    }

    public static Vector2d calculateVector(double x1 , double y1 , double x2 , double y2)
    {
        controller.kp=kp;
        controller.ki=ki;
        controller.kd=kd;
        double X = (x2-x1)*(x2-x1);
        double Y= (y2-y1)*(y2-1);

        double distance=Math.sqrt(X+Y);

        double cosinus = (x2-x1)/distance;
        double sinus = (y2-y1)/distance;

        double power= controller.calculate(0 , distance);

        double xFinal= cosinus*power;
        double yFinal=sinus*power;

        return new Vector2d(xFinal , yFinal);
    }
}