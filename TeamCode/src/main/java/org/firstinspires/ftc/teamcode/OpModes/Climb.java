package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Robot.Hardware;

@TeleOp(group = "z")
public class Climb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Hardware.init(hardwareMap);

        Differential.init();
        Wheelie wheelie=new Wheelie();
        PTO pto=new PTO();
        waitForStart();


        while(opModeIsActive())
        {
            if(gamepad1.dpad_up)Differential.liftPower=1;
            else if(gamepad1.dpad_down)Differential.liftPower=-1;
            else Differential.liftPower=0;

            if(gamepad1.right_bumper){wheelie.goDown();wheelie.up=false;wheelie.goDown=true;}
            else if(gamepad1.left_bumper){wheelie.goDown();wheelie.up=true;wheelie.goDown=true;}
            else wheelie.goDown=false;

            if(gamepad1.dpad_left)pto.setState("climb");
            if(gamepad1.dpad_right)pto.setState("normal");


            pto.update();
            Differential.update();
            wheelie.update();



        }
    }
}
