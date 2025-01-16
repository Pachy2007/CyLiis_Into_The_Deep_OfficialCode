package org.firstinspires.ftc.teamcode.Math;

import com.qualcomm.robotcore.util.ElapsedTime;

public class BetterMotionProfile {

    public double maxVelocity , acceleration , deceleration;
    double initialPosition;
    public double finalPosition;
    double initialVelocity;
    public ElapsedTime timer=new ElapsedTime();
    double distance , highVelocity , distance1 , distance3 , distance2 , t1 , t3 , t2 , position , velocity;
    public BetterMotionProfile(double maxVelocity , double acceleration , double deceleration)
    {
        this.maxVelocity=maxVelocity;
        this.acceleration=acceleration;
        this.deceleration=deceleration;
        timer.startTime();
    }
    public void setMotion(double initialPosition , double finalPositon , double initialVelocity)
    {
        this.initialPosition=initialPosition;
        this.finalPosition=finalPositon;
        this.initialVelocity=initialVelocity;
        position=initialPosition;
        distance=Math.abs(finalPositon-initialPosition);
        velocity=initialVelocity;
        highVelocity=Math.min(maxVelocity , Math.sqrt( (2*distance*acceleration*deceleration + initialVelocity*initialVelocity*deceleration) / (acceleration+deceleration) ) );
        distance1=(highVelocity+initialVelocity)*(highVelocity-initialVelocity)/(2*acceleration);
        distance3=highVelocity*highVelocity/(2*deceleration);
        distance2=Math.max(0 , distance-distance1-distance3);

        t1=(highVelocity-initialVelocity*Math.signum(finalPositon - initialPosition))/acceleration;
        t3=highVelocity/deceleration;
        t2=Math.max( 0 , distance2/highVelocity );
        timer.reset();
    }
    private void updatePosition()
    {
        if(0<=timer.seconds() && timer.seconds()<=t1)position=initialPosition+Math.signum(finalPosition-initialPosition)*(initialVelocity*timer.seconds()*Math.signum(finalPosition-initialPosition)+acceleration*timer.seconds()*timer.seconds()/2);
        else if(t1<timer.seconds() && timer.seconds()<=t1+t2)position=initialPosition+Math.signum(finalPosition-initialPosition)*(distance1+highVelocity*(timer.seconds()-t1));
        else if(t1+t2<timer.seconds() && timer.seconds()<=t1+t2+t3)position=initialPosition+Math.signum(finalPosition-initialPosition)*(distance1+distance2+highVelocity*(timer.seconds()-t1-t2)-deceleration*(timer.seconds()-t1-t2)*(timer.seconds()-t1-t2)/2);
        else position=finalPosition;
    }
    public double getPosition()
    {
        return position;
    }
    public double getVelocity()
    {
        return velocity;
    }
    public void updateVelocity()
    {
        if(0<=timer.seconds() && timer.seconds()<=t1)velocity=initialVelocity+acceleration*timer.seconds();
        else if(t1<timer.seconds() && timer.seconds()<=t1+t2)velocity=highVelocity;
        else if(t1+t2<timer.seconds() && timer.seconds()<=t1+t2+t3)velocity=highVelocity-deceleration*(timer.seconds()-t1-t2);
        else velocity=0;
    }
    public double getTimeToMotionEnd()
    {
        return Math.max(0 , t1+t2+t3-timer.seconds());
    }
    public void update()
    {
        updatePosition();
        updateVelocity();
    }
}