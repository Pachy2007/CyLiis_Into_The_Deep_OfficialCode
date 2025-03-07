package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

public class Pose2D {
    public double x;
    public double y;
    public double heading;

    Pose2D correction;

    public Pose2D(double x , double y , double heading)
    {
        this.x=x;
        this.y=y;
        this.heading=heading;
    }

    public void correct()
    {
        x+=correction.x;
        y+=correction.y;
        heading+=correction.heading;
    }

    public Pose2D(double x , double y , double heading ,  Pose2D correction)
    {
        this.x=x;
        this.y=y;
        this.heading=heading;
        this.correction=correction;
    }
    public Pose2D(){}

}