package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake.State.INTAKE;

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


         double X = 0;
         double Y = 0;

        Hardware.init(hardwareMap);
        Limelight3A limelight;
        limelight=hardwareMap.get(Limelight3A.class , "LimeLight");

        boolean INTAKE=false;
        Extendo extendo=new Extendo();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        DigitalChannel bb;

        bb=hardwareMap.get(DigitalChannel.class , "bb");
        Intake intake=new Intake();

        limelight.pipelineSwitch(0);
        limelight.reloadPipeline();
        limelight.start();

        Odo.init(hardwareMap , telemetry);

        LLResult result;
        result=limelight.getLatestResult();
        waitForStart();

        Point [] SRC = {
                new Point(-10.13,23.85),
                new Point( -39.78, 26.99),
                new Point( -21.36,28.97),
                new Point( 5.73, 29.58)
        };
        Point[] DST =  {
                new Point( -8, 36),
                new Point( -38, 45),
                new Point( -16 , 51),
                new Point( 14, 52)
        };

        while(opModeIsActive())
        {
            if(gamepad1.options)Odo.reset();

            if(!INTAKE)
             result=limelight.getLatestResult();

            if(result.getTx()==0)
            driveTrain.setTargetPosition(0 , 0 , 0);

            if(result.getTx()!=0 && !INTAKE)
            {
                MatOfPoint2f src = new MatOfPoint2f(), dst = new MatOfPoint2f();
                src.fromArray(SRC);
                dst.fromArray(DST);
                Mat status = new Mat();
                Mat Homog = Calib3d.findHomography(src, dst, Calib3d.RANSAC, 10, status);

                MatOfPoint2f idfk = new MatOfPoint2f();
                Point[] P = { new Point(result.getTx(), result.getTy()) };
                idfk.fromArray(P);

                MatOfPoint2f idfkdst = new MatOfPoint2f();
                Core.perspectiveTransform(idfk, idfkdst, Homog);

                P = idfkdst.toArray();
                X = P[0].x;
                Y = P[0].y;


                intake.setState(Intake.State.REPAUS_UP);
                extendo.setIn();
                angle=X*kp-(X-last)*kd;
            lastY=Y;


           // if(driveTrain.inPosition(10 , 10 , 0.08))
            //driveTrain.setTargetSpecialPosition(0 , 0 , Odo.getHeading()+Math.atan(X/Y));

            double last=X;
            //if(Math.abs(result.getTx())<4)INTAKE=true;
            }
            if(INTAKE)
            {
               // driveTrain.setTargetPosition(0 , 0 , lastHeading);
                if(bb.getState())
                {extendo.setTargetPosition(lastY*kExtend);
                if(extendo.inPosition())intake.setState(Intake.State.INTAKE_DOWN);}

                if(!bb.getState()){
                    extendo.setIn();
                    if(extendo.state!= Extendo.State.IN)
                    intake.setState(Intake.State.INTAKE_UP);
                    else intake.setState(Intake.State.REPAUS_UP);

                }
            }


            telemetry.addData("X" , X);
            telemetry.addData("Y" , Y);

            if(result!=null){
            telemetry.addData("XX" , result.getTx());
            telemetry.addData("YY" , result.getTy());}
            telemetry.addData("unghi" , Math.atan(X/Y));

            telemetry.update();

            driveTrain.update();
            extendo.update();
            intake.update();
            Odo.update();

            controller.kp=kp;
            controller.ki=ki;
            controller.kd=kd;
        }
    }
}
