package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {


    public DcMotorEx encoder;
    public  double reverse=1;


    public Encoder(DcMotorEx encoder , boolean reversed)
    {
        this.encoder=encoder;
        if(reversed)reverse=-1;
    }

    public void resetPosition()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPosition()
    {
        return encoder.getCurrentPosition()/50*reverse;
    }

    public double getVelocity(){return encoder.getVelocity()*reverse;}

}