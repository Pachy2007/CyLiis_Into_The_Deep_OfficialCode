package org.firstinspires.ftc.teamcode.Wrappers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {


    public DcMotorEx encoder;


    public Encoder(DcMotorEx encoder , boolean reversed)
    {
        this.encoder=encoder;
        if(reversed)encoder.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void resetPosition()
    {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public double getPosition()
    {
        return encoder.getCurrentPosition()/50;
    }

    public double getVelocity(){return encoder.getVelocity();}

}