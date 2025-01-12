package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Arm;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

public class SampleAutoNodes {

    public static Pose2D beforePutSamplePosition=new Pose2D (200 , 900 ,-2.24);
    public static Pose2D putSamplePosition=new Pose2D (130 , 1040 ,-2.24);

    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{ new Pose2D(277 ,950 , -2.97) , new Pose2D(234 ,950 , -3.27) , new Pose2D(296 ,950 , -3.623)};
    public static Pose2D parkPosition=new Pose2D(1700 , 80 , Math.PI/2);

    public static double[] takeFloorSampleExtendoPosition={820 , 830 , 810};

    public static double timeTakeSamples=1.7;
    public static double timeToScore=0.5;
    public MecanumDriveTrain driveTrain;
    Intake intake;
    public Outtake outtake;
    Extendo extendo;
    Latch latch;


    DigitalChannel bb;
    public boolean INIT=false;

    ElapsedTime timerTakeSamples;
    ElapsedTime timer;
    Node putSample , takefromFloor , park;
    public Node currentNode;
    boolean take=false;
    boolean a=true;
    public void run(HardwareMap hardwareMap , Telemetry telemetry)
    {

        if(!INIT)
        {
            timer=new ElapsedTime();
            INIT=true;
            putSample=new Node("putSample");
            takefromFloor=new Node("takeFromFloor");
            park = new Node("park");

            bb=hardwareMap.get(DigitalChannel.class , "bb");
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            extendo=new Extendo();
            latch=new Latch();

            outtake.haveSample=true;

            timer.reset();
            putSample.addConditions(
                    ()->{

                        if(putSample.index==0)
                        {
                            if(driveTrain.targetX!=140)
                                driveTrain.setTargetPosition(beforePutSamplePosition);
                            if(outtake.claw.state==outtake.claw.states.get("close") && driveTrain.inPosition() && outtake.state== Outtake.State.DeafultWithElement){outtake.goForHigh();outtake.goUp();timer.reset();}

                            if(outtake.lift.inPosition() && outtake.state== Outtake.State.Up && (outtake.lift.state== Lift.State.UP || outtake.lift.state== Lift.State.GOING_UP))driveTrain.setTargetPosition(putSamplePosition);
                            if(outtake.lift.inPosition() && outtake.state== Outtake.State.Up && timer.seconds()>1 && driveTrain.inPosition()){timer.reset();a=false;}

                            if(timer.seconds()>0.5  && timer.seconds()<1 && outtake.lift.inPosition() && outtake.state== Outtake.State.Up && !a && driveTrain.inPosition())
                            {outtake.score();
                                a=true;}
                        }
                        else{
                            if(driveTrain.targetX!=140)
                            driveTrain.setTargetPosition(beforePutSamplePosition);
                            if(extendo.state!=Extendo.State.IN && extendo.state!=Extendo.State.GOING_IN)
                                extendo.setIn();
                            if(extendo.state!=Extendo.State.IN)intake.setState(Intake.State.INTAKE_UP);

                            if(extendo.state== Extendo.State.IN && !bb.getState())
                                outtake.grabSample();
                            if(outtake.claw.state==outtake.claw.states.get("close") && driveTrain.inPosition()){outtake.goForHigh();outtake.goUp();}

                            if(outtake.lift.inPosition() && outtake.state== Outtake.State.Up && (outtake.lift.state== Lift.State.UP || outtake.lift.state== Lift.State.GOING_UP))driveTrain.setTargetPosition(putSamplePosition);
                            if(outtake.lift.inPosition() && outtake.state== Outtake.State.Up && timer.seconds()>1 && driveTrain.inPosition()){timer.reset();a=false;}

                                if(timer.seconds()>0.5  && timer.seconds()<1 && outtake.lift.inPosition() && outtake.state== Outtake.State.Up && !a && driveTrain.inPosition())
                                {outtake.score();
                                    a=true;}
                        }
                    }
                    ,
                    ()->{
                        return ( outtake.state== Outtake.State.Deafult && bb.getState() && a);
                    }
                    ,
                    new Node[]{takefromFloor , takefromFloor , takefromFloor , park}
            );

            takefromFloor.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takefromFloor.index]);

                        outtake.goDefault();
                        if(driveTrain.inPosition(30 , 30 , 0.08    )){
                        intake.setState(Intake.State.INTAKE_DOWN);
                        extendo.setTargetPosition(takeFloorSampleExtendoPosition[takefromFloor.index]);}
                    }
                    ,
                    ()->{

                        return !bb.getState() || timer.seconds()>7.5;
                    }
                    ,
                    new Node[]{putSample}
            );

            park.addConditions(
                    ()->{

                        intake.setState(Intake.State.REPAUS_UP);

                        Outtake.prim=true;
                        Outtake.highChamberDown=415;
                        outtake.haveSample=false;
                        outtake.state= Outtake.State.Up;
                        driveTrain.setTargetPosition(parkPosition);

                    }
                    ,()->{
                        return false;
                    }
                    ,
                    new Node[]{}
            );

            currentNode=putSample;
        }

        currentNode.run();

        extendo.update();
        driveTrain.update();
        intake.update();
        outtake.update();
        latch.update();
        Odo.update();


        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];
    }

}
