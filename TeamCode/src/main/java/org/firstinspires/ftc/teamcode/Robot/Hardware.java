package org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.io.BufferedReader;

public class Hardware {

    public static boolean INIT=false;

    public static DcMotorEx mch0 , mch1 , mch2 , mch3;

    public static DcMotorEx meh0 , meh1 , meh2 , meh3;
    public static ColorSensor colorSensor;

    public static Servo sch4 ;
    public static Servo sch5;
    public static Servo sch0 , sch1;
    public static Servo sch2;
    public static Servo sch3;
    public static Servo seh0 , seh1 , seh2 , seh3 , seh4 , seh5;

    public static Servo ssh0 , ssh1 , ssh2 , ssh3 , ssh4 , ssh5;

    public static DigitalChannel depositBeamBreak;
    public static DigitalChannel extendoBeamBreak;
    public static RevColorSensorV3 distanceSnsor;

    public static double IMUOFFSET=0;


    public static AdafruitBNO055IMU imu;

    public static void unlock(DcMotorEx motor)
    {
        MotorConfigurationType mct = motor.getMotorType();
        mct.setAchieveableMaxRPMFraction(1);
        motor.setMotorType(mct);
    }

    public static void init(HardwareMap hardwareMap)
    {
        IMUOFFSET=0;
         distanceSnsor=hardwareMap.get(RevColorSensorV3.class , "colorSensor");
        colorSensor=hardwareMap.get(ColorSensor.class , "colorSensor");
        depositBeamBreak=hardwareMap.get(DigitalChannel.class , "depositBB");
           extendoBeamBreak=hardwareMap.get(DigitalChannel.class , "extendoBeamBreak");
        imu=hardwareMap.get(AdafruitBNO055IMU.class , "imu");

        mch0=hardwareMap.get(DcMotorEx.class , "ch0");
        mch1=hardwareMap.get(DcMotorEx.class , "ch1");
        mch2=hardwareMap.get(DcMotorEx.class , "ch2");
        mch3=hardwareMap.get(DcMotorEx.class , "ch3");


        //unlock(mch0);
        //unlock(mch1);
       // unlock(mch2);
        //unlock(mch3);


        meh0=hardwareMap.get(DcMotorEx.class , "eh0");
        meh1=hardwareMap.get(DcMotorEx.class , "eh1");
        meh2=hardwareMap.get(DcMotorEx.class , "eh2");
        meh3=hardwareMap.get(DcMotorEx.class , "eh3");

        //unlock(meh0);
        //unlock(meh1);
        //unlock(meh2);
        //unlock(meh3);



        sch0=hardwareMap.get(Servo.class , "sch0");
        sch1=hardwareMap.get(Servo.class , "sch1");
        sch2=hardwareMap.get(Servo.class , "sch2");
        sch3=hardwareMap.get(Servo.class , "sch3");
        sch4=hardwareMap.get(Servo.class , "sch4");
        sch5=hardwareMap.get(Servo.class , "sch5");

        ssh0=hardwareMap.get(Servo.class , "ssh0");
        ssh1=hardwareMap.get(Servo.class , "ssh1");
        ssh2=hardwareMap.get(Servo.class , "ssh2");
        ssh3=hardwareMap.get(Servo.class , "ssh3");
        ssh4=hardwareMap.get(Servo.class , "ssh4");
        ssh5=hardwareMap.get(Servo.class , "ssh5");


        seh0=hardwareMap.get(Servo.class , "seh0");
        seh1=hardwareMap.get(Servo.class , "seh1");
        seh2=hardwareMap.get(Servo.class , "seh2");
        seh3=hardwareMap.get(Servo.class , "seh3");
        seh4=hardwareMap.get(Servo.class , "seh4");
        seh5=hardwareMap.get(Servo.class , "seh5");






    }

}