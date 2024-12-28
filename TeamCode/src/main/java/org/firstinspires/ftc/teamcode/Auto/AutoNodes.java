package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.fasterxml.jackson.databind.ser.std.StdKeySerializers;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.OpModes.TeleOp;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;
import org.firstinspires.ftc.teamcode.Wrappers.Pose2D;

@Config
public class AutoNodes {


    public static Pose2D putSpecimenPosition=new Pose2D (860 , 300 ,0);

    public static Pose2D takeFloorSpecimenPosition1=new Pose2D(625 ,-335 , -2.2);
    public static Pose2D takeFloorSpecimenPosition2=new Pose2D(600 ,-340 , -2.02);
    public static Pose2D takeFloorSpecimenPosition3=new Pose2D(600 ,-630 , -1.95);

    public static Pose2D releaseSpecimenPosition1=new Pose2D(625 ,-335 ,-0.7);
    public static Pose2D releaseSpecimenPosition2=new Pose2D(600 ,-340 ,-0.7);
    public static Pose2D releaseSpecimenPosition3=new Pose2D(600 ,-550 ,-0.7);

    public static Pose2D beforeTakeWallSpecimenPosition=new Pose2D(140 , -440 , -3.14);


    public static Pose2D takeWallSpecimenPosition=new Pose2D(2 ,-440 ,-3.14);

    public static Pose2D takeSampleHumanPosition=new Pose2D(0 ,0 ,0);

    public static Pose2D scoreBascketPosition=new Pose2D(0,0,0);



    public static double takeSpecimenPosition1=570;
    public static double takeSpecimenPosition2=850;
    public static double takeSpecimenPosition3=830;

    public static double releaseSpecimenExtendo=430;
    public static double timeTakeSamples=1.7;
    public MecanumDriveTrain driveTrain;
    Intake intake;
    public Outtake outtake;
    Extendo extendo;
    Latch latch;


    DigitalChannel bb;
    public boolean INIT=false;

    ElapsedTime timerTakeSamples;
    ElapsedTime timer;
    Node putSpecimen , takeFloorSpecimen , releaseSpecimen , takeWallSpecimen , takeSampleHuman , scoreBasket , takeSample , park;
    public Node currentNode;
    boolean take=false;
    public void run(HardwareMap hardwareMap , Telemetry telemetry)
    {

        if(!INIT)
        {
            putSpecimenPosition=new Pose2D(860 , 300 ,0);

            timer=new ElapsedTime();
            timerTakeSamples=new ElapsedTime();
            putSpecimen=new Node("putSpecimen");
            takeFloorSpecimen=new Node("takeFloorSpecimen");
            releaseSpecimen=new Node("releaseSpecimen");
            takeWallSpecimen=new Node("takeWallSpecimen");

            Hardware.init(hardwareMap);
            INIT=true;

            bb=hardwareMap.get(DigitalChannel.class , "bb");
            driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.PID);
            intake=new Intake();
            outtake=new Outtake(Outtake.State.DeafultWithElement);
            extendo=new Extendo();
            latch=new Latch();


            ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerAuto;


            outtake.haveSample=false;
            Outtake.prim=true;

            putSpecimen.addConditions(
                    ()->{

                        driveTrain.setTargetPosition(putSpecimenPosition);
                        outtake.goUp();
                        if((driveTrain.inPosition(30 , 400 , 0.06) || (Odo.odo.getVelX()<0.1 && Odo.getX()>750)) && outtake.inPosition())outtake.score();
                        take=false;
                    }
                    ,
                    ()->{
                        if(outtake.claw.state==outtake.claw.states.get("open"))timerTakeSamples.reset();
                        return outtake.claw.state==outtake.claw.states.get("open");
                    }
                    ,
                    new Node[]{takeFloorSpecimen , takeWallSpecimen}
            );

            takeFloorSpecimen.addConditions(
                    ()->{
                        if(takeFloorSpecimen.index==0)
                        driveTrain.setTargetPosition(takeFloorSpecimenPosition1);

                        if(takeFloorSpecimen.index==1)
                        driveTrain.setTargetPosition(takeFloorSpecimenPosition2);

                        if(takeFloorSpecimen.index==2)
                        driveTrain.setTargetPosition(takeFloorSpecimenPosition3);

                        Outtake.prim=false;
                        if(driveTrain.inPosition())
                        {
                            if(takeFloorSpecimen.index==0)
                            extendo.setTargetPosition(takeSpecimenPosition1);
                            if(takeFloorSpecimen.index==1)
                                extendo.setTargetPosition(takeSpecimenPosition2);
                            if(takeFloorSpecimen.index==2)
                                extendo.setTargetPosition(takeSpecimenPosition3);

                        intake.setState(Intake.State.INTAKE_DOWN);}
                        else timerTakeSamples.reset();


                    }
                    ,
                    ()->{
                        return !bb.getState() || timerTakeSamples.seconds()>timeTakeSamples;
                    }
                    ,
                    new Node[]{releaseSpecimen , releaseSpecimen , releaseSpecimen}
            );

            releaseSpecimen.addConditions(
                    ()->{
                        extendo.setTargetPosition(releaseSpecimenExtendo);
                        if(releaseSpecimen.index==0)
                            driveTrain.setTargetPosition(releaseSpecimenPosition1);
                        if(releaseSpecimen.index==1)
                            driveTrain.setTargetPosition(releaseSpecimenPosition2);
                        if(releaseSpecimen.index==2)
                            driveTrain.setTargetPosition(releaseSpecimenPosition3);

                        if(driveTrain.inPosition(70 , 70 ,0.13) && timer.seconds()>1)
                        {intake.setState(Intake.State.REVERSE_DOWN);
                        timer.reset();
                        }
                        take=false;

                    }
                    ,
                    ()->{
                        if(bb.getState()&& timer.seconds()>0.8 && intake.state== Intake.State.REVERSE_DOWN){outtake.takeSpecimen(); timerTakeSamples.reset();}
                        return  bb.getState()&& timer.seconds()>0.8 && intake.state== Intake.State.REVERSE_DOWN;
                    }
                    ,
                    new Node[]{takeFloorSpecimen , takeFloorSpecimen , takeWallSpecimen}
            );

            takeWallSpecimen.addConditions(
                    ()->{
                        extendo.setIn();

                        if(driveTrain.targetX!=2)
                        driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition);

                        if(driveTrain.inPosition(100 , 10 , 0.08))
                        driveTrain.setTargetPosition(takeWallSpecimenPosition);

                        if(takeWallSpecimen.index<4)
                        outtake.takeSpecimen();
                        else outtake.goDefault();
                        intake.setState(Intake.State.REPAUS_UP);

                        if(driveTrain.inPosition(30 , 50 , 0.09)&&  outtake.inPosition() && !take && outtake.state==Outtake.State.Specimen && driveTrain.targetX==2){outtake.grabSample();take=true;}
                    }
                    ,
                    ()->{
                        if(outtake.state==Outtake.State.DeafultWithElement && outtake.claw.inPosition()){putSpecimenPosition.y+=65;
                                                                                                        if(takeWallSpecimen.index==0)
                                                                                                        putSpecimenPosition.x+=20;
                                                                                                        else putSpecimenPosition.x+=3;

                            putSpecimenPosition.heading=-3.14;
                        return true;}
                        return false;
                    }
                    ,
                    new Node[]{putSpecimen , putSpecimen , putSpecimen , putSpecimen , takeWallSpecimen}
            );

            currentNode=putSpecimen;

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
