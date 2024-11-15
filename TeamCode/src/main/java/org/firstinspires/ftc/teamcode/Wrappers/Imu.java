package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Imu {

    public  static GoBildaPinpointDriver  odo;
    public static double heading , x ,y;
    public  static void init(HardwareMap hardwareMap)
    {
        odo=hardwareMap.get(GoBildaPinpointDriver.class , "odo");

        odo.resetPosAndIMU();
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
        odo.resetPosAndIMU();
    }

    public  static void update()
    {
        odo.update();
        heading=odo.getHeading();
        x=odo.getPosX();
        y=odo.getPosY();
    }
}