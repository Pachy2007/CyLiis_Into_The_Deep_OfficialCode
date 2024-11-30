package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Odo {

    public  static GoBildaPinpointDriver  odo;
    public static double heading , x ,y;
    public static boolean INIT=false;

    public static Telemetry telemetry;
    public  static void init(HardwareMap hardwareMap , Telemetry telemetryy)
    {
        if(INIT)return;
            telemetry=telemetryy;
        INIT=true;
        odo=hardwareMap.get(GoBildaPinpointDriver.class , "odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(125.5 , 0);
        odo.resetPosAndIMU();
        odo.recalibrateIMU();
    }

    public  static void init(HardwareMap hardwareMap , Telemetry telemetryy , String string)
    {
        telemetry=telemetryy;
        INIT=true;
        odo=hardwareMap.get(GoBildaPinpointDriver.class , "odo");

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setOffsets(125.5 , 0);
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
        try {
            odo.update();
            heading=odo.getHeading();
            x=odo.getPosX();
            y=odo.getPosY();

        } catch (Exception ignored){
            telemetry.addLine(ignored.getMessage());
        }
        }
}