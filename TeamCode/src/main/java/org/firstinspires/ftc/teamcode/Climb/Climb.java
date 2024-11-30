package org.firstinspires.ftc.teamcode.Climb;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robot.Hardware;

public class Climb {


    CRServo servo1;
    CRServo servo2;


    public Climb()
    {
        servo1= Hardware.sch0;
        servo2=Hardware.sch1;
        servo1.setDirection(DcMotorSimple.Direction.REVERSE);

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
    public void Constant()
    {
        servo1.setPower(-0.25);
        servo2.setPower(-0.25);
    }
}
