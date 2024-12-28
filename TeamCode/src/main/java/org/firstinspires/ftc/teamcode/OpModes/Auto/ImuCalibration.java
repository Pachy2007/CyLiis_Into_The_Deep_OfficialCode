package org.firstinspires.ftc.teamcode.OpModes.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Wrappers.Odo;

import java.util.Timer;

@Autonomous
public class ImuCalibration extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        Odo.init(hardwareMap , telemetry , "a");

        Odo.calibrate();

        ElapsedTime timer=new ElapsedTime();

        while(opModeInInit())
        {
            if(timer.seconds()>2)return;
        }
    }
}
