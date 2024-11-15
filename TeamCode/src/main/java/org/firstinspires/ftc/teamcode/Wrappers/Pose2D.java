package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    public Pose2D(double x , double y , double heading)
    {
        this.x=x;
        this.y=y;
        this.heading=heading;
    }
    public Pose2D(){}

    @Override
    public String toString(){
        return String.valueOf(x) + " " + String.valueOf(y) + " " + String.valueOf(heading);
    }
}