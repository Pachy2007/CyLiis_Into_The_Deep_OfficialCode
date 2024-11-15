package org.firstinspires.ftc.teamcode.Modules.Drive;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Math.PidAngle;
import org.firstinspires.ftc.teamcode.Math.PidToPoint;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Imu;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

import java.util.Vector;

public class MecanumDriveTrain {

    public enum State{
        DRIVE ,
        PID
    } State state;

    Vector2d vector;
    double angular;

    Pose2D targetPosition;



    public static boolean frontLeftreversed=false , frontRightreversed=true , backLeftreversed=false , backRightreversed=true;

    BetterMotor frontLeft , frontRight , backLeft , backRight;
    public double getHeading(){
        return Imu.getHeading();
    }

    public MecanumDriveTrain(State state)
    {

        this.state=state;

        frontLeft=new BetterMotor(Hardware.mch3 , BetterMotor.RunMode.RUN , frontLeftreversed);
        frontRight=new BetterMotor(Hardware.mch0 , BetterMotor.RunMode.RUN , frontRightreversed);

        backLeft=new BetterMotor(Hardware.mch2 , BetterMotor.RunMode.RUN , backLeftreversed);
        backRight=new BetterMotor(Hardware.mch1 , BetterMotor.RunMode.RUN , backRightreversed);

        vector=new Vector2d(0 , 0);
        angular=0;
        //PidToPoint.init();
        //PidAngle.init();
    }


    public void setTargetPosition(Pose2D position)
    {
        targetPosition=position;
    }

    public void setPower(double x , double y , double rotation)
    {
        vector=new Vector2d(x ,y);
        angular=rotation;
    }


    private void updateHardware()
    {
        double heading=Imu.getHeading();
        double sinus=Math.sin(-heading);
        double cosinus=Math.cos(-heading);

        double X=vector.getX();
        double Y=vector.getY();

        double x=cosinus*X - sinus*Y;
        double y=sinus*X + cosinus*Y;

        double dom=Math.max(1 , Math.abs(x)+Math.abs(y)+Math.abs(angular));
        frontLeft.setPower((y+x+angular)/dom);

        frontRight.setPower((y-x-angular)/dom);

        backLeft.setPower((y-x+angular)/dom);

        backRight.setPower((y+x-angular)/dom);

    }

    public void update()
    {

        if(state==State.PID){
        vector= PidToPoint.calculateVector(
                Imu.getX() , Imu.getY() ,
                targetPosition.x , targetPosition.y
                );
        angular= PidAngle.calculate(Imu.getHeading() , targetPosition.heading);}

        updateHardware();
    }
}