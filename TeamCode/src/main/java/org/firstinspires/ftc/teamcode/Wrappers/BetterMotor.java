package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Math.PIDController;

public class BetterMotor {

    public enum RunMode{
        RUN ,
        PID
    }
    RunMode runMode;

    PIDController pidController;
    public DcMotorEx motor;
    boolean power;
    DcMotorEx encoder;

    public double targetPosition;
    public int encoderReversed=-1;

    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed , DcMotorEx encoder)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public BetterMotor(DcMotorEx motor , RunMode runMode , boolean reversed , DcMotorEx encoder , boolean encoderReversed)
    {
        this.motor=motor;
        this.runMode=runMode;

        this.encoder=encoder;
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        if(encoderReversed)this.encoderReversed=-1;
        else this.encoderReversed=1;

        if(reversed)this.motor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setPidCoefficients(double kp , double ki , double kd)
    {
        pidController=new PIDController(kp , ki , kd);
    }
    public void setPidCoefficients(double kp , double ki , double kd , double ff1 , double ff2)
    {
        pidController=new PIDController(kp , ki , kd , ff1 , ff2);
    }

    public void setPosition(double position)
    {
        power=false;
        targetPosition=position;
    }

    public void setPower(double power)
    {
        this.power=true;
        motor.setPower(power);
    }
    public void setRunMode(RunMode runMode)
    {
        this.runMode=runMode;
    }

    public double getPosition()
    {
        return encoder.getCurrentPosition() * encoderReversed;
    }

    public double getVelocity()
    {
        return encoder.getVelocity() * encoderReversed;
    }

    public void resetPosition()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void update()
    {
        if(runMode==RunMode.RUN || power)return;

        double power=pidController.calculate(targetPosition , encoder.getCurrentPosition()*encoderReversed);

        motor.setPower(power);


    }
}