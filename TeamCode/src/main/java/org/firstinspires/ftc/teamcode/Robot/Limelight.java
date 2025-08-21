package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Limelight {

    public static Limelight3A limelight;
    public static double angle=30;
    public static double Height=285;
    public static double distanceIntake=93;
    public static double lateralDistance=-103;
    public static LLResult result;
    public static double k=0.9;
    public static double Distance;
    public static double r=27;
    public static double targetAngle;
    public static double distanceLimeLight=74.5;

    public static double extendoPosition;


    public static double X;
    public static double Y;

    public static void init(HardwareMap hardwareMap , int pipeline){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public static void update(){
        result=limelight.getLatestResult();
        if(result==null)return;


        Y=Height*Math.tan(Math.toRadians(result.getTy()+90-angle));

        X=Math.sqrt( Y*Y + Height*Height)*Math.tan(Math.toRadians(result.getTx()));


        targetAngle=Math.atan((X+lateralDistance)/(Y+distanceLimeLight));

        Distance = Math.sqrt( (X+lateralDistance)*(X+lateralDistance) +(Y+distanceLimeLight)*(Y+distanceLimeLight));
        extendoPosition=(Distance-distanceIntake)/(2*Math.PI*r)*(8194/50);
    }

}

