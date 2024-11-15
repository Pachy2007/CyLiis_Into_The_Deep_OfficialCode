package org.firstinspires.ftc.teamcode.Climb;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Climb {


    CRServo servo1;
    CRServo servo2;


    public Climb()
    {
        servo1= Hardware.sch4;
        servo2=Hardware.sch5;

    }

    public void goUp()
    {
        servo1.setPower(1);
        servo2.setPower(1);
    }



    public void goDown()
    {
        servo1.setPower(-1);
        servo2.setPower(-1);
    }

    public void Default()
    {
        servo1.setPower(0);
        servo2.setPower(0);
    }
}
