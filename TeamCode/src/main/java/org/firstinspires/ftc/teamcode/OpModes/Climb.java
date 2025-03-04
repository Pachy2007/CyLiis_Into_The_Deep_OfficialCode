package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp(group = "z")
public class Climb extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        Differential.init();
        Wheelie wheelie=new Wheelie();
        PTO pto=new PTO();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        waitForStart();


        while(opModeIsActive())
        {
            double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );

            if(gamepad1.options) Odo.reset();

            if(gamepad1.dpad_up)Differential.liftPower=1;
            else if(gamepad1.dpad_down)Differential.liftPower=-1;
            else Differential.liftPower=0;



             if(gamepad1.left_bumper){wheelie.goUp();}
            if(gamepad1.right_bumper) wheelie.goDown();

            if(gamepad1.dpad_left)pto.setState("climb");
            if(gamepad1.dpad_right)pto.setState("normal");


            driveTrain.update();
            pto.update();
            Differential.update();
            telemetry.addData("motor1" , Hardware.meh1.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor2" , Hardware.mch2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor3" , Hardware.mch3.getCurrent(CurrentUnit.AMPS));
            telemetry.update();



        }
    }
}
