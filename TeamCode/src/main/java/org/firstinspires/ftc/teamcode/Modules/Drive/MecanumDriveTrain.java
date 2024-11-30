package org.firstinspires.ftc.teamcode.Modules.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.BetterMotor;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

@Config
public class MecanumDriveTrain {



    public enum State{
        DRIVE , PID , CLIMB;
    }
    State state;

    BetterMotor frontLeft , frontRight;
    BetterMotor backLeft , backRight;

    public  double targetX , targetY ;
            public static double targetHeading;
    public static double error;
    double rotation;

    public static boolean frontLeftreversed=false , frontRightreversed=true , backLeftreversed=false , backRightreversed=true;

    public static double lateralMultiplier=2;
    public static  double realHeading;

    public static double kp=0.01 , ki=0 , kd=0.0012;
    public static double KP=2 , KI , KD=0.21;
    PIDController controllerX=new PIDController(kp , ki , kd) , controllerY=new PIDController(kp , ki , kd) , controllerHeading=new PIDController(KP , KI , KD);

    public MecanumDriveTrain(State initialState)
    {
        state=initialState;

        frontLeft=new BetterMotor(Hardware.mch3 , BetterMotor.RunMode.RUN , frontLeftreversed);
        frontRight=new BetterMotor(Hardware.mch2 , BetterMotor.RunMode.RUN , frontRightreversed);

        backLeft=new BetterMotor(Hardware.mch0 , BetterMotor.RunMode.RUN , backLeftreversed);
        backRight=new BetterMotor(Hardware.mch1 , BetterMotor.RunMode.RUN , backRightreversed);

    }
    public boolean inPosition()
    {
        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        error=targetHeading-realHeading;
        if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));

        if(Math.abs(targetX-Odo.getX())<50 && Math.abs(targetY-Odo.getY())<50 && Math.abs(error)<0.06)return true;
        return false;
    }
    public boolean inPosition( double x , double y , double error)
    {
        double heading= Odo.getHeading();
        if(heading<0)realHeading=Math.abs(heading);
        else realHeading=2*Math.PI-heading;

        error=targetHeading-realHeading;
        if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));

        if(Math.abs(targetX-Odo.getX())<x && Math.abs(targetY-Odo.getY())<y && this.error<error)return true;
        return false;
    }

    public void setTargetVector(double x , double y , double rx)
    {
        x*=lateralMultiplier;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        frontRight.setPower(frontRightPower);
        backRight.setPower(backRightPower);

    }
    public void setMode(State state)
    {
        this.state=state;
    }
    public void setTargetPosition(double x , double y , double heading)
    {
        targetX=x;
        targetY=y;
        targetHeading=heading-Math.floor((heading/ (Math.PI*2)))*Math.PI*2;


    }
    public void setTargetPosition(Pose2d position)
    {
        targetX=position.getX();
        targetY=position.getY();
        targetHeading=position.getHeading();


    }
    public void setTargetPosition(Pose2D position)
    {
        targetX=position.x;
        targetY=position.y;
        targetHeading=position.heading;
    }


    public void update()
    {
        controllerX.kp=kp;
        controllerY.kp=kp;

        controllerX.ki=ki;
        controllerY.ki=ki;

        controllerX.kd=kd;
        controllerY.kd=kd;

        controllerHeading.kp=KP;
        controllerHeading.ki=KI;
        controllerHeading.kd=KD;





            double x;
            double y;




            x = controllerX.calculate(targetX, Odo.getX());



            y=-controllerY.calculate(targetY , Odo.getY());

            double heading= Odo.getHeading();
            if(heading<0)realHeading=Math.abs(heading);
            else realHeading=2*Math.PI-heading;

            error=targetHeading-realHeading;
            if(Math.abs(error)>Math.PI)error=-Math.signum(error)*(2*Math.PI-Math.abs(error));
            rotation= controllerHeading.calculate(error , 0);


            setTargetVector(y*Math.cos(-heading) - x*Math.sin(-heading) , y*Math.sin(-heading)+x*Math.cos(-heading) , rotation);


    }


}