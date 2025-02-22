package org.firstinspires.ftc.teamcode.Utils;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Robot.Hardware;


@Config
@TeleOp(group = "b")
public class HubTest  extends LinearOpMode {


    enum HUB{
        CONTROL_HUB("ch") , EXPANSION_HUB("eh") , SERVO_HUB("sh");

        String hub;
        HUB(String string)
        {
            hub=string;
        }

    }

    enum HARDWARE{
        MOTOR("") , SERVO("s") , ENCODER("");

        String hardware;
        HARDWARE(String string)
        {
            hardware=string;
        }
    }

    enum PORT{
        _0("0") , _1("1") , _2("2") , _3("3") , _4("4") , _5("5");

        String port;

        PORT(String string)
        {
            port=string;
        }
    }


    public static HUB hub=HUB.CONTROL_HUB;
    public static HARDWARE hardware=HARDWARE.MOTOR;
    public static PORT port=PORT._0;


    DcMotorEx motor;
    Servo servo;

    public static double power=0 , position=0;

    @Override
    public void runOpMode() throws InterruptedException {
            waitForStart();



            while (opModeIsActive())
            {
                String string= hardware.hardware+ hub.hub+port.port;
                switch (hardware)
                {
                    case ENCODER:
                    case MOTOR:
                        motor=hardwareMap.get(DcMotorEx.class , string);
                        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                        break;

                    case SERVO:
                        servo=hardwareMap.get(Servo.class , string);
                        break;
                }

                if(hardware==HARDWARE.ENCODER)telemetry.addData("Position" , motor.getCurrentPosition());
                telemetry.update();

                if(hardware == HARDWARE.MOTOR) motor.setPower(power);
                if(hardware == HARDWARE.SERVO) servo.setPosition(position);
            }


    }






}