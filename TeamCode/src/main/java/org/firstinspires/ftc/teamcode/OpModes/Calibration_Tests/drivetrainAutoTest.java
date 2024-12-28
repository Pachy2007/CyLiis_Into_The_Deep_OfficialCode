package org.firstinspires.ftc.teamcode.OpModes.Calibration_Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
@TeleOp(group = "z")
public class drivetrainAutoTest extends LinearOpMode {

    public static double x1=200 , y1=-400 , heading1=3.14;
    public static double x2=2500 , y2=-400 , heading2;
    public static double x3=2500 , y3=2000 , heading3=3.14;
    public static double x4=200 , y4=2000 , heading4;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);

        boolean ok1=false , ok2=false , ok3=false , ok4=false;

        waitForStart();

        while(opModeIsActive())
        {
            if(ok1==false)
            driveTrain.setTargetPosition(x1 , y1 , heading1);

            if(ok1==false && driveTrain.inPosition())
            {ok1=true;}

            if(ok2==false && ok1==true)driveTrain.setTargetPosition(x2 , y2 , heading2);

            if(ok2==false && driveTrain.inPosition() && ok1==true)ok2=true;

            if(ok2==true && ok3==false)driveTrain.setTargetPosition(x3 , y3 , heading3);
            if(ok3==false && ok2==true && driveTrain.inPosition())ok3=true;

            if(ok3==true)driveTrain.setTargetPosition(x4 , y4 , heading4);
            if(ok3==true && driveTrain.inPosition()){ok1=false; ok2=false; ok3=false;}


            telemetry.addData("x" , Odo.getX());
            telemetry.addData("y" , Odo.getY());
            telemetry.addData("heading" , Odo.getHeading());


            telemetry.update();

            Odo.update();
            driveTrain.update();
        }
    }
}
