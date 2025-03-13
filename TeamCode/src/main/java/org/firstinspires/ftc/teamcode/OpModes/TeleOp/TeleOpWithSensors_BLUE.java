package org.firstinspires.ftc.teamcode.OpModes.TeleOp;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Others.Scissor;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "pachy")
public class TeleOpWithSensors_BLUE extends LinearOpMode {

    enum State{
        CONTROLLING , CLIMB , SCORING_SPECIMEN;
    }
    State state= State.CONTROLLING;


    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D[] beforePutSpecimenPosition = {
            new org.firstinspires.ftc.teamcode.Wrappers.Pose2D( 865 ,-125 ,-0.45)
    };
    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D[] putSpecimenPosition = {
            new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(890 ,0 ,-0.45)
    };

    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D beforeTakeWallSpecimenPosition[] ={
            new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(180 ,-750 ,-0.65)
    };

    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D[] takeWallSpecimenPosition = {
            new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(-5 ,-720 ,0)
    };

    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D[] afterTakeWallSpecimenPosition={
            new org.firstinspires.ftc.teamcode.Wrappers.Pose2D( 50 , -600 , -0.5)
    };


    public static double minL = 35, maxL = 85;

    @Override
    public void runOpMode() throws InterruptedException {



        if(Odo.INIT)
            Odo.odo.setPosition(new Pose2D(DistanceUnit.MM , 0 ,0 , AngleUnit.RADIANS , Odo.getHeading()-Hardware.IMUOFFSET));

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);


         double L=0;

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean prevB=false;


        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Intake intake=new Intake(SampleColor.State.BLUE , true);
        Scissor scissor = new Scissor();

        PTO pto=new PTO();
        Wheelie wheelie=new Wheelie();
        boolean climb=false;

        outtake.extension.setState("goRetrect");
        outtake.arm.setState("goHighSpecimen");

        Node prepareClimbLvl2 , climbLvl2 , prepareClimbLvl3 , climbLvl3 , readyToBeStopped;
        Node climbState;


        Node beforeTakeWallSpecimen , takeWallSpecimen , afterTakeWallSpecimen , beforePutSpecimen , putSpecimen;
        Node scoringSpecimenState;

        afterTakeWallSpecimen = new Node("afterTakeWallSpecimen");
        beforeTakeWallSpecimen = new Node("prepareToTakeSpecimen");
        takeWallSpecimen = new Node("takeWallSpecimen");
        beforePutSpecimen = new Node("beforePutSpecimen");
        putSpecimen = new Node("putSpecimen");

        beforeTakeWallSpecimen.addConditions(
                ()->{
                    intake.setState(Intake.State.REPAUS_UP);
                    intake.setExtendoIN();
                    outtake.takeSpecimen();
                    driveTrain.setTargetPosition(beforeTakeWallSpecimenPosition[Math.min(beforeTakeWallSpecimenPosition.length-1 , beforeTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return outtake.arm.inPosition() &&driveTrain.inPosition(400 , 300 , 0.2);
                }
                ,
                new Node[]{takeWallSpecimen}
        );

        takeWallSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeWallSpecimenPosition[Math.min(takeWallSpecimenPosition.length-1 , takeWallSpecimen.index)]);
                    if(driveTrain.inPosition(25 , 150 ,0.5) || (Odo.getX()<50 && Odo.odo.getVelX()<15))
                        outtake.grabSample();
                }
                ,
                ()->{
                    return outtake.claw.state==outtake.claw.states.get("closeSpecimen") || outtake.claw.state==outtake.claw.states.get("goCloseSpecimen");
                }
                ,
                new Node[]{afterTakeWallSpecimen}
        );
        afterTakeWallSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(afterTakeWallSpecimenPosition[Math.min(afterTakeWallSpecimenPosition.length-1 , afterTakeWallSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(1000 , 1000 , 0.25);
                }
                ,
                new Node[]{beforePutSpecimen}
        );

        beforePutSpecimen.addConditions(
                ()->{
                    outtake.goUp();
                    driveTrain.setTargetPosition( beforePutSpecimenPosition[Math.min(beforePutSpecimenPosition.length-1 , beforePutSpecimen.index)]);
                }
                ,
                ()->{
                    return driveTrain.inPosition(300 , 350 , 0.6) || (Odo.odo.getVelX()<15 && Odo.getX()>550);
                }
                ,
                new Node[]{putSpecimen}
        );

        putSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(putSpecimenPosition[Math.min(putSpecimenPosition.length-1 , putSpecimen.index)]);
                    if(driveTrain.inPosition(40 , 100 , 0.6) || (Odo.odo.getVelX()<15 && Odo.getX()>550) )outtake.score();
                }
                ,
                ()->{
                    return driveTrain.inPosition(150 , 250 , 0.4) && outtake.state!= Outtake.State.Up || (Odo.odo.getVelX()<15 && Odo.getX()>550);
                }
                ,
                new Node[]{beforeTakeWallSpecimen}

        );

        scoringSpecimenState=beforeTakeWallSpecimen;

        prepareClimbLvl2 = new Node("prepareClimbLvl2");
        climbLvl2 = new Node("climbLvl2");
        prepareClimbLvl3 = new Node("prepareClimbLvl3");
        climbLvl3 = new Node("climbLvl3");
        readyToBeStopped = new Node("readyToBeStopped");
        ElapsedTime readyToBeStoppedTimer=new ElapsedTime();
        prepareClimbLvl2.addConditions(
                ()->{
                    Differential.motor1.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Differential.motor2.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    Differential.boostLift.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                    wheelie.goUp();
                    outtake.climb2();
                }
                ,
                ()->{
                    return wheelie.inPosition() && outtake.inPosition();
                }
                ,
                new Node[]{climbLvl2}
        );

        climbLvl2.addConditions(
                ()->{
                    if(outtake.lift.encoder.getPosition()<100)wheelie.goDown();
                    pto.setState("goClimb");
                    outtake.goDefault();
                }
                ,
                ()->{
                    if(Lift.State.DOWN==outtake.lift.state && readyToBeStoppedTimer.seconds()>1.2)readyToBeStoppedTimer.reset();
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN && readyToBeStoppedTimer.seconds()>0.8 && readyToBeStoppedTimer.seconds()<1.2)
                    {readyToBeStoppedTimer.reset();return true;}
                    return false;
                }
                ,
                new Node[]{prepareClimbLvl3}
        );
        prepareClimbLvl3.addConditions(
                ()->{
                    if(!outtake.lift.inPosition())
                    intake.extendo.setTargetPosition(200);
                    if(outtake.lift.encoder.getPosition()>150)
                    wheelie.goUp();
                    if(outtake.arm.state!=outtake.arm.states.get("goFinalClimb3") || outtake.arm.state!=outtake.arm.states.get("finalClimb3"))
                    outtake.arm.setState("climb");
                    if(readyToBeStoppedTimer.seconds()>0.15)
                      outtake.climb3();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state!= Lift.State.DOWN)
                    {
                        outtake.arm.setState("goFinalClimb3");
                    }
                    if(outtake.lift.inPosition())intake.extendo.setIn();
                    return  outtake.lift.inPosition() && outtake.lift.state!= Lift.State.DOWN && Math.abs(Hardware.imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.RADIANS))<0.3 && intake.extendo.state== Extendo.State.IN && intake.extendoBlocker.state==intake.extendoBlocker.states.get("close");
                }
                ,
                new Node[]{climbLvl3}
        );
        climbLvl3.addConditions(
                ()->{
                    pto.setState("goClimb");
                    if(intake.extendo.state== Extendo.State.IN)
                    outtake.goDefault();
                    wheelie.goUp();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN)
                    {
                        Differential.liftPower=-1;
                        readyToBeStoppedTimer.reset(); return true;}
                    return false;
                }
                ,
                new Node[]{readyToBeStopped}
        );
        readyToBeStopped.addConditions(
                ()->{
                    outtake.climb=true;
                        Differential.liftPower=-1;
                    outtake.arm.setState("climb");
                    if(readyToBeStoppedTimer.seconds()>0.05)
                        wheelie.goDown();


                }
                ,()->{
                    return false;
                }
                ,
                new Node[]{readyToBeStopped}
        );
        climbState=prepareClimbLvl2;

        while(opModeInInit())
        {
            outtake.extension.update();
            outtake.arm.update();
            intake.update();
            pto.update();
            Odo.update();
            wheelie.update();
            gamepad1.setLedColor(44, 128, 14 , 2100000000);
            gamepad2.setLedColor(163, 139, 191, 2100000000);
        }
        waitForStart();

        while(opModeIsActive()){


            if(gamepad1.dpad_up){
                state= State.CLIMB;
            }

                if(state== State.CONTROLLING)
            {
                readyToBeStoppedTimer.reset();
                driveTrain.setMode(MecanumDriveTrain.State.DRIVE);
                double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

             L = ((intake.extendo.encoder.getPosition()/750.0) * (maxL-minL) + minL)/minL;
            rotation = rotation;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );

            if(intake.extendo.encoder.getPosition()>100 && intake.extendo.state!= Extendo.State.IN && intake.extendo.state!= Extendo.State.GOING_IN)scissor.setState("goingDeploied");
            else scissor.setState("goingRetrected");

            if(gamepad1.options)Odo.reset();

            if(gamepad1.a)intake.setExtendoIN();

            if(gamepad2.y && outtake.state!=Outtake.State.TakingElement)outtake.goDefault();

            if(((gamepad2.a || (intake.stateTransfer== Intake.TransferLogic.ReadyToTransfer && intake.extendo.state== Extendo.State.IN)) && !intake.justColorAllaiance) && outtake.state== Outtake.State.Deafult && outtake.inPosition() && intake.ramp.state==intake.ramp.states.get("up"))outtake.grabSample();

            if(gamepad1.circle && outtake.state==Outtake.State.Specimen)outtake.grabSample();

            if(gamepad2.x)outtake.retry(); //failsafePusSpecimen

            if(gamepad2.dpad_up){outtake.goUp();outtake.goForHigh();}
            if(gamepad2.dpad_down){outtake.goUp();outtake.goForLow();}



            if((gamepad1.circle) && (outtake.state==Outtake.State.DeafultWithElement || outtake.state== Outtake.State.ReleaseSample) && outtake.haveSample==true)outtake.releaseSample();
            if(gamepad1.circle)outtake.score();
            if(gamepad2.circle)outtake.takeSpecimen();

            if(gamepad2.left_bumper)intake.justColorAllaiance=false;
            if(gamepad2.right_bumper)intake.justColorAllaiance=true;

            if(gamepad1.right_bumper)intake.setState(Intake.State.INTAKE_DOWN);
            else if(gamepad1.left_bumper){intake.setState(Intake.State.REVERSE_DOWN);ActiveIntake.State.REVERSE.power=ActiveIntake.reversePowerTeleOp;}
            else intake.setState(Intake.State.REPAUS_UP);

            double power=-gamepad1.right_stick_y;
            intake.setExtendoVelocity(power);

            }

            if(state== State.CLIMB)
            {
                if(gamepad1.dpad_down){Differential.liftPower=0;climb=true;}
                else if(!climb){
                climbState.run();
                if(climbState.transition())climbState=climbState.next[Math.min(climbState.index++ , climbState.next.length-1)];}
            }

            if(gamepad1.y && !prevB)
            {
                if(state== State.CONTROLLING)
                {state= State.SCORING_SPECIMEN;
                    scoringSpecimenState=takeWallSpecimen;
                    Odo.odo.setPosition(new Pose2D(DistanceUnit.MM , 0 , -730 , AngleUnit.RADIANS , 0));
                }
                else state= State.CONTROLLING;
            }
            prevB=gamepad1.y;

            if(state== State.SCORING_SPECIMEN && !gamepad1.y)
            {
                driveTrain.setMode(MecanumDriveTrain.State.PID);
                scoringSpecimenState.run();
                if(scoringSpecimenState.transition())scoringSpecimenState=scoringSpecimenState.next[Math.min(scoringSpecimenState.index++ , scoringSpecimenState.next.length-1)];

            }

            telemetry.addData("motorWithGear" , Hardware.meh2.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorWithBelt" , Hardware.mch3.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motorBoost" , Hardware.mch2.getCurrent(CurrentUnit.AMPS));

            telemetry.update();


            driveTrain.update();
            outtake.update();
            intake.update();
            Odo.update();
            pto.update();
            wheelie.update();

        }
    }
}

