package org.firstinspires.ftc.teamcode.OpModes;

import android.icu.util.Measure;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.drive.Drive;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

public class Limelight {

    public static Limelight3A limelight;
    public static double angle=20;
    public static double Height=308.88;
    public static double distanceIntake=105;
    public static double lateralDistance=-113;
    public static LLResult result;
    public static double k=1.4;
    public static double Distance;

    public static double extendoPosition;


    public static double X;
    public static double Y;

    public static void init(HardwareMap hardwareMap){
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
    }

    public static void update(){
        result=limelight.getLatestResult();
        Y=Height*Math.tan(Math.toRadians(result.getTy()+90-angle))-distanceIntake;
        X=Height*Math.tan(Math.toRadians(result.getTy()+90-angle))*Math.tan(Math.toRadians(result.getTx()))-lateralDistance;
         Distance=Math.sqrt(X*X+Y*Y);
        extendoPosition=k*Distance;
    }

}

