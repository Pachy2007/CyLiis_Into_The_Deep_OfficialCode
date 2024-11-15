package org.firstinspires.ftc.teamcode.Wrappers;


public class Math {


    public static double ColorDistance(Color color1 , Color color2)
    {
        double red= color1.red-color2.red;
        double green=color1.green-color2.green;
        double blue=color1.blue-color2.blue;

        double distance= java.lang.Math.sqrt(red*red+green*green+blue*blue);

        return distance;
    }


}
