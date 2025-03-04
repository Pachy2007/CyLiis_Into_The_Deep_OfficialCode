package org.firstinspires.ftc.teamcode.Modules.Others;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.network.ApChannel;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.State;
import org.firstinspires.ftc.teamcode.Wrappers.BetterServo;

@Config
public class Wheelie {


    public static double ssh5Normal=0.8 , ssh5Up=0.17 ;
    public static double ssh4Normal=0.35 , ssh4Up=0.97;
    BetterServo servo4 , servo5;
    public static double maxVelocity=10 , acc=10;
    public Wheelie()
    {
        servo4= new BetterServo("ssh4" , Hardware.ssh0 ,  BetterServo.RunMode.PROFILE ,  ssh4Normal, false);
        servo5= new BetterServo("ssh5" , Hardware.ssh1 ,  BetterServo.RunMode.PROFILE ,  ssh5Normal, false);

        servo4.setProfileCoefficients(maxVelocity , acc , acc);
        servo5.setProfileCoefficients(maxVelocity , acc , acc);

    }
    public void goUp()
    {
        servo4.setPosition(ssh4Up);
        servo5.setPosition(ssh5Up);
        update();
    }

    public void goDown()
    {
        servo4.setPosition(ssh4Normal);
        servo5.setPosition(ssh5Normal);
        update();
    }

    public boolean inPosition()
    {
        return servo4.inPosition() && servo5.inPosition();
    }

    public void update()
    {
        servo4.update();
        servo5.update();
    }
}
