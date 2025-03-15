package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Robot.Limelight;


@TeleOp(group = "z")
@Config
public class TestSampleDetection extends LinearOpMode {

    public static boolean extend;
    public static double kp=0 , ki=0     , kd=0 , kX , lastY , kExtend=27;
    public double angle , last;
    double lastHeading;
    PIDController controller=new PIDController(kp , ki , kd);
    @Override
    public void runOpMode() throws InterruptedException {


        Limelight.init(hardwareMap , 0);

        waitForStart();

        while(opModeIsActive())
        {

            Limelight.update();

            telemetry.addData("X" , Limelight.X);
            telemetry.addData("Y" , Limelight.Y);
            telemetry.addData("targetHeading" , Limelight.targetAngle);
            telemetry.update();

        }
    }
}
