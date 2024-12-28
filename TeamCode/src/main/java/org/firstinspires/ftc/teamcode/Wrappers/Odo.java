package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.opencv.core.Mat;

import java.lang.Math;

public class Odo {

    public  static GoBildaPinpointDriver  odo;
    public static double heading , x ,y;
    public static boolean INIT=false;

    public static Telemetry telemetry;
    public static boolean plsMergi=false;
    public  static void init(HardwareMap hardwareMap , Telemetry telemetryy)
    {
        if(INIT)return;
        telemetry=telemetryy;
        INIT=true;
        odo=hardwareMap.get(GoBildaPinpointDriver.class , "odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(125.5 , 0);
        odo.resetPosAndIMU();

       // if(!plsMergi)
        {
        odo.recalibrateIMU();plsMergi=true;}
    }

    public  static void init(HardwareMap hardwareMap , Telemetry telemetryy , String string)
    {

        if(INIT){odo.setPosition(new Pose2D(DistanceUnit.MM , 0 , 0 , AngleUnit.RADIANS , 0));return;}
        telemetry=telemetryy;
        INIT=true;
        odo=hardwareMap.get(GoBildaPinpointDriver.class , "odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(125.5 , 0);

        if(!plsMergi)
        {
        odo.recalibrateIMU();plsMergi=true;}
    }

    public static void calibrate()
    {
        odo.recalibrateIMU();
    }

    public static double getHeading()
    {
        return heading;
    }

    public static double getX()
    {
        return x;
    }

    public static double getY()
    {
        return y;
    }

    public static void reset()
    {
        odo.setPosition(new Pose2D(DistanceUnit.MM , 0 , 0 , AngleUnit.RADIANS , 0));
    }

    public  static void update()
    {
            odo.update();
            heading=odo.getHeading();
            x=odo.getPosX();
            y=odo.getPosY();

            if(Math.abs(heading)> Math.PI)heading=heading-2*Math.PI*Math.signum(heading);

        }
}