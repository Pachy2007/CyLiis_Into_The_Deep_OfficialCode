package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;
import org.opencv.core.Mat;

@Config
public class SpecimenAutoNodes {

    public static double[] intakeExtendoPosition={540 ,800 ,1050};
    public static double[] reverseExtendoPosition={650 ,650 ,650};

    public static Pose2D beforePutSpecimenPosition=new Pose2D (-600 , 30 ,0);
    public static Pose2D putSpecimenPosition=new Pose2D(-805 ,30 ,0);
    public static Pose2D[] takeFloorSamplePosition=new Pose2D[]{
            new Pose2D(-500 ,850 ,0.364),
            new Pose2D(-500 ,850 ,0.704),
            new Pose2D(-500 ,850,0.964)
    };
    public static Pose2D[] releaseSamplePosition=new Pose2D[]{
            new Pose2D(-500, 850 ,2.2),
            new Pose2D(-500, 850 ,2.2),
            new Pose2D(-500, 850 ,2.2)
    };
    public static Pose2D beforeTakeWallSpecimenPosition=new Pose2D(-155 ,690 ,0);
    public static Pose2D takeWallSpecimenPosition=new Pose2D(-5 ,690 ,0);
    public boolean skip=false;



    ElapsedTime timerOuttake;
    Node beforePutSpecimen , putSpecimen , takeFloorSample , releaseSample , beforeTakeWallSpecimen ,takeWallSpecimen;

    public MecanumDriveTrain driveTrain;
    public Intake intake;
    public Outtake outtake;
    public Extendo extendo;
    public Latch latch;

    DigitalChannel bb;

    public boolean INIT=false;

    ElapsedTime timer;
    ElapsedTime reverseTimer;

    Node currentNode;
    public void run(HardwareMap hardwareMap , Telemetry telemetry)
    {
        if(!INIT)
        {
            beforePutSpecimenPosition=new Pose2D (-580 , 20 ,0);
            putSpecimenPosition=new Pose2D(-805 ,20 ,0);

            beforeTakeWallSpecimen=new Node("beforeTakeWallSpecimen");
            beforePutSpecimen=new Node("beforePutSpecimen");
            putSpecimen=new Node("putSpecimen");
            takeFloorSample=new Node("takeFloorSample");
            releaseSample=new Node("releaseSample");
            takeWallSpecimen=new Node("takeWallSpecimen");

            Hardware.init(hardwareMap);
            INIT=true;

            bb=hardwareMap.get(DigitalChannel.class , "bb");
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            outtake.haveSample=false;
            extendo=new Extendo();
            latch=new Latch();

            timerOuttake=new ElapsedTime();
            reverseTimer=new ElapsedTime();
            timer=new ElapsedTime();



            beforePutSpecimen.addConditions(
                    ()->{

                        if(outtake.arm.state==outtake.arm.states.get("withElementSpecimen"))
                        {
                            if(beforePutSpecimen.index==0)Outtake.highChamberDown=500;
                            else Outtake.highChamberDown=510;
                            outtake.goUp();}
                        driveTrain.setTargetPosition(beforePutSpecimenPosition);
                    }
                    ,
                    ()->{
                        return driveTrain.inPosition(20 , 8 , 0.1) && outtake.inPosition();
                    }
                    ,
                    new Node[]{putSpecimen}
            );

            putSpecimen.addConditions(
                    ()->{
                        driveTrain.setTargetPosition(putSpecimenPosition);
                    }
                    ,
                    ()->{
                        timer.reset();
                        if(driveTrain.inPosition() || (Math.abs(Odo.odo.getVelX())<0.1 && driveTrain.x<-700)){
                            outtake.score();putSpecimenPosition.y-=75;beforePutSpecimenPosition.y-=75;return true;}
                        return false;

                    }
                    ,
                    new Node[]{takeFloorSample , beforeTakeWallSpecimen}
            );

            takeFloorSample.addConditions(
                    ()->{
                        outtake.goDefault();
                        driveTrain.setTargetPosition(takeFloorSamplePosition[takeFloorSample.index]);
                        if(driveTrain.inPosition())
                        {extendo.setTargetPosition(intakeExtendoPosition[takeFloorSample.index]);
                        intake.setState(Intake.State.INTAKE_DOWN);}
                    }
                    ,
                    ()->{
                        if(timer.seconds()>3){return true;}
                        return !bb.getState();
                    }
                    ,
                    new Node[]{releaseSample}

            );
            releaseSample.addConditions(
                    ()->{
                        if(skip)return;
                        timer.reset();
                        driveTrain.setTargetPosition(releaseSamplePosition[releaseSample.index]);
                        if(driveTrain.inPosition() || releaseSample.index>0)
                        extendo.setTargetPosition(reverseExtendoPosition[releaseSample.index]);

                        if(driveTrain.inPosition() && extendo.inPosition())intake.setState(Intake.State.REVERSE_DOWN);
                    }
                    ,
                    ()->{
                        if(skip){timer.reset();skip=false;return true;}
                        if(!bb.getState() && reverseTimer.seconds()>1 && driveTrain.inPosition() && extendo.inPosition())reverseTimer.reset();
                        if(reverseTimer.seconds()>0.5 && reverseTimer.seconds()<1){extendo.setTargetPosition(300);return true;}
                        return false;
                    }
                    ,
                    new Node[]{takeFloorSample , takeFloorSample , beforeTakeWallSpecimen}
            );
            beforeTakeWallSpecimen.addConditions(
                    ()->{
                        extendo.setIn();
                        driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition);

                        if(outtake.lift.state==Lift.State.DOWN)
                        outtake.takeSpecimen();
                        intake.setState(Intake.State.REPAUS_UP);
                    }
                    ,
                    ()->{

                        return (outtake.inPosition() && driveTrain.inPosition() && outtake.state== Outtake.State.Specimen && outtake.arm.state==outtake.arm.states.get("takeSpecimen"));
                    }
                    ,
                    new Node[]{takeWallSpecimen}
            );

            takeWallSpecimen.addConditions(
                    ()->{

                        if(takeWallSpecimen.index>=4)
                        {driveTrain.setTargetPosition(takeWallSpecimenPosition);
                            outtake.goDefault();
                            intake.setState(Intake.State.REPAUS_UP);
                        }
                        else{
                        driveTrain.setTargetPosition(takeWallSpecimenPosition);
                        if(driveTrain.inPosition(8 , 40 , 0.1) || (Math.abs(Odo.odo.getVelX())<0.1 && driveTrain.x>-10))outtake.grabSample();}
                    }
                    ,
                    ()->{
                        if(outtake.claw.state==outtake.claw.states.get("closeSpecimen")){timerOuttake.reset();return true;}
                        return false;
                    }
                    ,
                    new Node[]{beforePutSpecimen , beforePutSpecimen , beforePutSpecimen , beforePutSpecimen , takeWallSpecimen}
            );

            currentNode=beforePutSpecimen;
        }
        else {
        currentNode.run();

        if(currentNode.transition())currentNode=currentNode.next[Math.min(currentNode.index++ , currentNode.next.length-1)];



        Odo.update();
        driveTrain.update();
        intake.update();
        extendo.update();
        outtake.update();
        latch.update();}

    }




}
