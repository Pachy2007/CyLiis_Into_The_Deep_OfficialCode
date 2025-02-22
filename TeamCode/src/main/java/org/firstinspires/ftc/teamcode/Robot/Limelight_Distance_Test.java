package org.firstinspires.ftc.teamcode.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;


@TeleOp
@Config
public class Limelight_Distance_Test extends LinearOpMode {


    Limelight3A limelight;
    public static double angle=20;
    public static double Height=308.88;
    public static double distanceIntake=105;
    public static double lateralDistance=-115;

    public static double k=1.45;
    MecanumDriveTrain driveTrain;
    Extendo extendo;
    ElapsedTime timer=new ElapsedTime();
    DigitalChannel bb;

    public static boolean A=false;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        bb=hardwareMap.get(DigitalChannel.class , "bb");
        limelight.start();
        double X=0 ,Y=0;

        Odo.init(hardwareMap , telemetry);
        Hardware.init(hardwareMap);

        driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
        Intake intake=new Intake(SampleColor.State.RED , true);
        extendo=new Extendo();
        timer=new ElapsedTime();
        boolean z=false;
        boolean second=false;

        waitForStart();

        while (opModeIsActive())
        {
            LLResult result = limelight.getLatestResult();

            if(!A)
            {driveTrain.setTargetPosition(0 ,0 ,0);
             Y=Height*Math.tan(Math.toRadians(result.getTy()+90-angle))-distanceIntake;
             X=Height*Math.tan(Math.toRadians(result.getTy()+90-angle))*Math.tan(Math.toRadians(result.getTx()))-lateralDistance;
            extendo.setIn();
            intake.setState(Intake.State.REPAUS_DOWN);
            timer.reset();
            }
            if(A==true)
            {
                if(timer.seconds()<3){
                driveTrain.setTargetPosition(0 ,0 , Math.atan(X/Y));
                if(extendo.state== Extendo.State.IN){intake.setState(Intake.State.REPAUS_UP);intake.update();}
                if(driveTrain.inPosition() && intake.ramp.inPosition())extendo.setTargetPosition(Math.sqrt(X*X+Y*Y)*k);
                else timer.reset();
                if(extendo.inPosition() && extendo.state!= Extendo.State.IN)intake.setState(Intake.State.INTAKE_DOWN);
                z=false;
                second=false;}

                if(timer.seconds()>3 && bb.getState())
                {
                    if(z==false)
                    {extendo.setIn();z=true; intake.setState(Intake.State.REVERSE_DOWN);}
                    if(extendo.state== Extendo.State.IN)
                        intake.setState(Intake.State.REPAUS_UP);
                    intake.update();
                    if(!second && intake.state== Intake.State.REPAUS_UP && intake.ramp.inPosition()){
                        extendo.setTargetPosition(300);
                        if(extendo.inPosition()){extendo.setIn();second=true;}
                    }

                    if(second){
                    if(intake.state== Intake.State.REPAUS_UP && intake.ramp.inPosition() && extendo.state== Extendo.State.IN && driveTrain.inPosition())extendo.setTargetPosition(Math.sqrt(X*X+Y*Y)*k);
                    if(extendo.state!= Extendo.State.IN && extendo.inPosition())intake.setState(Intake.State.INTAKE_DOWN);}
                }
            }

            telemetry.addData("y" , Y/10);
            telemetry.addData("x" , X/10);
            telemetry.update();
            driveTrain.update();
            extendo.update();
            intake.update();
            Odo.update();
        }



    }
}
