package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Math.PIDController;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.OpModes.Climb;
import org.firstinspires.ftc.teamcode.OpModes.Limelight;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;


@TeleOp
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
            telemetry.update();

        }
    }
}
