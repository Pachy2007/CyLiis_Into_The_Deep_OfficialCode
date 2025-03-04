package org.firstinspires.ftc.teamcode.Wrappers;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Math.BetterMotionProfile;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class BetterServo {

    public enum RunMode{
        PROFILE,
        Time,
        GoToPosition;
    }

    public ElapsedTime timer=new ElapsedTime();
    public double time;

    public RunMode runMode;

    public BetterMotionProfile profile;

    public double position;

    public boolean INIT=false;
    public boolean reachedPosition=true;

    public Servo servo;
    public String name;

    public BetterServo(String name,   Servo servo , RunMode runMode , double initialPosition , boolean reversed)
    {
        this.name=name;
        this.servo=servo;
        this.runMode=runMode;
        this.servo.setPosition(initialPosition);
        if(reversed)this.servo.setDirection(Servo.Direction.REVERSE);
        else this.servo.setDirection(Servo.Direction.FORWARD);
        position=initialPosition;
    }
    public BetterServo(String name,   Servo servo , RunMode runMode , double initialPosition , boolean reversed , double time)
    {
        this.name=name;
        this.servo=servo;
        this.runMode=runMode;
        this.servo.setPosition(initialPosition);
        if(reversed)this.servo.setDirection(Servo.Direction.REVERSE);
        else this.servo.setDirection(Servo.Direction.FORWARD);
        position=initialPosition;
        if(runMode==RunMode.Time)
        {
            timer.startTime();
            this.time=time;
        }
    }
    public void setProfileCoefficients(double maxVelocity , double acceleration , double deceleration)
    {
        if(runMode!=RunMode.PROFILE)return;

        if(!INIT)
        {profile=new BetterMotionProfile(maxVelocity , acceleration , deceleration);INIT=true;}
        else {profile.maxVelocity=maxVelocity;
        profile.deceleration=deceleration;
        profile.acceleration=acceleration;}

        profile.setMotion(position , position , 0);
    }

    public void setPosition(double position)
    {
        if(runMode==RunMode.Time)if(this.position==position)return;
        if(runMode==RunMode.PROFILE)if(position==profile.finalPosition)return;



        switch (runMode)
        {
            case PROFILE:
                reachedPosition=false;
                profile.setMotion(profile.getPosition() , position , profile.getVelocity());
                break;
            case GoToPosition:
                reachedPosition=true;
                servo.setPosition(position);
                this.position=position;
                break;
            case Time:
                if(servo.getPosition()==position)return;
                reachedPosition=false;
                servo.setPosition(position);
                timer.reset();
                this.position=position;
                break;
        }
        update();
    }
    public void setMODE(RunMode mode)
    {
        runMode=mode;
    }



    public double getPosition()
    {
        return position;
    }

    public boolean inPosition()
    {
        if(runMode==RunMode.PROFILE)
        return reachedPosition;
        if(runMode==RunMode.Time)
            return timer.seconds()>time;
        return true;
    }

    public void update()    /* Add this function to the loop if you use motionprofile */
    {
        if(runMode==RunMode.PROFILE){
        profile.update();
        position=profile.getPosition();
        servo.setPosition(profile.getPosition());

        reachedPosition = profile.finalPosition == position;}



    }

}