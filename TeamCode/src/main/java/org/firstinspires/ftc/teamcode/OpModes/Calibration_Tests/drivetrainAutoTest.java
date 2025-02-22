package org.firstinspires.ftc.teamcode.OpModes.Calibration_Tests;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
@TeleOp(group = "z")
public class drivetrainAutoTest extends LinearOpMode {

    public static double x1=0 , y1=0 , heading1=0;


    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);


        waitForStart();

        while(opModeIsActive())
        {
            driveTrain.setTargetPosition(x1 , y1 , heading1);

            telemetry.addData("x" , Odo.getX());
            telemetry.addData("y" , Odo.getY());
            telemetry.addData("x velocity" , Odo.xVelocity);
            telemetry.addData("y velocity" , Odo.yVelocity);
            telemetry.addData("x robot velocity" , Odo.xRobotVelocity);
            telemetry.addData("y robot velocity" , Odo.yRobotVelocity);
            telemetry.addData("x glide", Odo.xGlide);
            telemetry.addData("y glide", Odo.yGlide);
            telemetry.addData("x predicted", Odo.predictedX);
            telemetry.addData("y predicted", Odo.predictedY);
            telemetry.addData("heading" , Odo.getHeading());

            telemetry.addData("targetX" , x1);
            telemetry.addData("targetY" , y1);
            telemetry.addData("targetHeading" , heading1);


            telemetry.update();

            Odo.update();
            driveTrain.update();
        }
    }
}
