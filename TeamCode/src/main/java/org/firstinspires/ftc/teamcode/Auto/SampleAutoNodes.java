package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class SampleAutoNodes {

    public static Pose2D putSamplePosition=new Pose2D (700 , -200 ,-1);

    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{
            new Pose2D(700 ,-200 , -1.3) ,
            new Pose2D(700 ,-200 , -1.6) ,
            new Pose2D(700 ,-200 , -1.9)};
    public static Pose2D parkPosition=new Pose2D(-50 , -1600 , 0);

    public static double[] takeFloorSampleExtendoPosition={970 , 880 , 910};

    public static double timeTakeSamples=2.5;
    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Extendo extendo;
    public Latch latch;


    ElapsedTime timer;

    DigitalChannel bb;
    public boolean INIT=false;
    Node goPutSample, putSample,  takefromFloor, park;

    public Node currentNode;
    boolean skip=false;

    boolean a=false;

    public void run(HardwareMap hardwareMap , Telemetry telemetry)
    {

        if(!INIT)
        {
            timer=new ElapsedTime();
            goPutSample = new Node("goPutSample");
            putSample = new Node("putSample");
            takefromFloor = new Node("takefromFloor");
            park = new Node("park");

            Hardware.init(hardwareMap);
            INIT=true;

            bb=hardwareMap.get(DigitalChannel.class , "bb");
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            outtake.haveSample=true;
            extendo=new Extendo();
            latch=new Latch();

            ElapsedTime timer=new ElapsedTime();

            goPutSample.addConditions(
                    ()->{
                        if(extendo.state== Extendo.State.IN)outtake.grabSample();
                        outtake.goUp();
                        driveTrain.setTargetPosition(putSamplePosition);
                    }
                    ,
                    ()->{
                        return (driveTrain.inPosition() && outtake.inPosition() && outtake.state==Outtake.State.Up) || skip;

                    }
                   ,
                   new Node[]{putSample}
            );
            putSample.addConditions(
                    ()->{
                        outtake.score();
                    }
                    ,
                    ()->{
                        timer.reset();
                        return (outtake.lift.state==Lift.State.GOING_DOWN) || skip;
                    }
                    ,
                    new Node[]{takefromFloor , takefromFloor , takefromFloor , park}
            );
            takefromFloor.addConditions(
                    ()->{
                        skip=false;
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takefromFloor.index]);
                        if(driveTrain.inPosition())
                        {
                            intake.setState(Intake.State.INTAKE_DOWN);
                            extendo.setTargetPosition(takeFloorSampleExtendoPosition[takefromFloor.index]);
                        }
                    }
                    ,
                    ()->{
                        if(timer.seconds()>timeTakeSamples){skip=true; return true;}
                         if(!bb.getState()){extendo.setIn();intake.setState(Intake.State.INTAKE_UP);return true;}
                             return false;
                    }
                    ,
                    new Node[]{goPutSample}
            );

            park.addConditions(
                    ()->{



                        if(!a){
                        intake.setState(Intake.State.REPAUS_UP);
                        driveTrain.setTargetPosition(parkPosition);
                        outtake.park();}

                        if(driveTrain.inPosition())
                        {
                            a=true;
                            driveTrain.setTargetVector(0 ,0 ,0);
                            Odo.odo.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D( DistanceUnit.MM, 0 , 0 , AngleUnit.RADIANS , -Math.PI/2));
                        }

                    }

                    ,
                    ()->{
                        return false;
                    }
                    ,
                    new Node[]{park}
            );


            currentNode=goPutSample;
        }

        else{
        currentNode.run();

        extendo.update();
        if(!a)
        driveTrain.update();
        intake.update();
        outtake.update();
        latch.update();
        Odo.update();


        telemetry.addData("currentNode" , currentNode.name);
        telemetry.addData("bb" , bb);
        telemetry.addData("outtake" , outtake.state);
        telemetry.addData("intake" , intake.state);
        telemetry.addData("X" , Odo.getX());
        telemetry.addData("Y" , Odo.getY());
        telemetry.addData("Heading" , Odo.getHeading());
        telemetry.addData("arm" , outtake.arm.state);
        telemetry.addData("claw" , outtake.claw.state);
        telemetry.update();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }

}
}
