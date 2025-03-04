package org.firstinspires.ftc.teamcode.OpModes;


import static org.firstinspires.ftc.teamcode.Wrappers.GoBildaPinpointDriver.DeviceStatus.READY;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.ActiveIntake;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Differential;
import org.firstinspires.ftc.teamcode.Modules.Others.PTO;
import org.firstinspires.ftc.teamcode.Modules.Intake.SampleColor;
import org.firstinspires.ftc.teamcode.Modules.Others.Wheelie;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "a")
public class TeleOpWithSensors_RED extends LinearOpMode {

    enum State{
        CONTROLLING , CLIMB , SCORING_SPECIMEN;
    }
    State state=State.CONTROLLING;


    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D prepareToTakeSpecimenPosition = new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(150 ,0 , 0);
    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D takeSpecimenPosition = new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(0 ,0 , 0);
    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D beforePutSpecimenPosition = new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(750 , 550 , 0);
    public static org.firstinspires.ftc.teamcode.Wrappers.Pose2D putSpecimenPosition = new org.firstinspires.ftc.teamcode.Wrappers.Pose2D(785 ,650 , -0.15);





    @Override
    public void runOpMode() throws InterruptedException {



        if(Odo.INIT)
            Odo.odo.setPosition(new Pose2D(DistanceUnit.MM , 0 ,0 , AngleUnit.RADIANS , Odo.getHeading()-Hardware.IMUOFFSET));

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);



        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean prevB=false;


        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Intake intake=new Intake(SampleColor.State.RED , true);

        PTO pto=new PTO();
        Wheelie wheelie=new Wheelie();

        outtake.extension.setState("goRetrect");
        outtake.arm.setState("goHighSpecimen");

        Node prepareClimbLvl2 , climbLvl2 , prepareClimbLvl3 , climbLvl3 , readyToBeStopped;
        Node climbState;


        Node prepareToTakeSpecimen , takeSpecimen , beforePutSpecimen , putSpecimen;
        Node scoringSpecimenState;

        prepareToTakeSpecimen = new Node("prepareToTakeSpecimen");
        takeSpecimen = new Node("takeSpecimen");
        beforePutSpecimen = new Node("beforePutSpecimen");
        putSpecimen = new Node("putSpecimen");





        prepareToTakeSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(prepareToTakeSpecimenPosition);
                    outtake.takeSpecimen();
                }
                ,
                ()->{
                    return driveTrain.inPosition(100 , 50 , 0.3) && outtake.inPosition();
                }
                ,
                new Node[]{takeSpecimen}
        );

        takeSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(takeSpecimenPosition);
                    if(driveTrain.inPosition(25 , 25 , 0.3))
                    outtake.grabSample();
                }
                ,
                ()->{
                    return outtake.claw.state==outtake.claw.states.get("goCloseSpecimen") || outtake.claw.state==outtake.claw.states.get("closeSpecimen");
                }
                ,
                new Node[]{beforePutSpecimen}
        );
        beforePutSpecimen.addConditions(
                ()->{
                    outtake.goUp();
                    driveTrain.setTargetPosition(beforePutSpecimenPosition);
                }
                ,
                ()->{
                    return outtake.inPosition() && outtake.state== Outtake.State.Up;
                }
                ,
                new Node[]{putSpecimen}
        );

        putSpecimen.addConditions(
                ()->{
                    driveTrain.setTargetPosition(putSpecimenPosition);
                    if(driveTrain.inPosition(30 , 100 , 0.2))outtake.score();
                }
                ,
                ()->{
                    return outtake.extension.state==outtake.extension.states.get("retrect");
                }
                ,
                new Node[]{prepareToTakeSpecimen}
        );

        scoringSpecimenState=prepareToTakeSpecimen;

        prepareClimbLvl2 = new Node("prepareClimbLvl2");
        climbLvl2 = new Node("climbLvl2");
        prepareClimbLvl3 = new Node("prepareClimbLvl3");
        climbLvl3 = new Node("climbLvl3");
        readyToBeStopped = new Node("readyToBeStopped");
        ElapsedTime readyToBeStoppedTimer=new ElapsedTime();
        prepareClimbLvl2.addConditions(
                ()->{
                    wheelie.goUp();
                    outtake.climb2();
                }
                ,
                ()->{
                    readyToBeStoppedTimer.reset();
                    return wheelie.inPosition() && outtake.inPosition();
                }
                ,
                new Node[]{climbLvl2}
        );

        climbLvl2.addConditions(
                ()->{
                    pto.setState("goClimb");
                    outtake.goDefault();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN)
                    {readyToBeStoppedTimer.reset();return true;}
                    return false;
                }
                ,
                new Node[]{prepareClimbLvl3}
        );
        prepareClimbLvl3.addConditions(
                ()->{
                    outtake.arm.setState("climb");
                    if(readyToBeStoppedTimer.seconds()>0.15)
                      outtake.climb3();
                }
                ,
                ()->{
                    return outtake.lift.inPosition() && outtake.lift.state!= Lift.State.DOWN;
                }
                ,
                new Node[]{climbLvl3}
        );
        climbLvl3.addConditions(
                ()->{
                    pto.setState("goClimb");
                    outtake.goDefault();
                    wheelie.goUp();
                }
                ,
                ()->{
                    if(outtake.lift.inPosition() && outtake.lift.state== Lift.State.DOWN)
                    {readyToBeStoppedTimer.reset(); return true;}
                    return false;
                }
                ,
                new Node[]{readyToBeStopped}
        );
        readyToBeStopped.addConditions(
                ()->{
                    outtake.lift.state= Lift.State.GOING_DOWN;
                    outtake.arm.setState("climb");
                    if(readyToBeStoppedTimer.seconds()>0.25)
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
            gamepad1.setLedColor(44, 128, 14 , 1000000000);
            gamepad2.setLedColor(163, 139, 191, 1000000000);
        }
        waitForStart();

        while(opModeIsActive()){


            if(gamepad1.dpad_up){state=State.CLIMB;}

                if(state==State.CONTROLLING)
            {driveTrain.setMode(MecanumDriveTrain.State.DRIVE);
                double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );



            if(gamepad1.a)intake.setExtendoIN();

            if(gamepad2.y && outtake.state!=Outtake.State.TakingElement)outtake.goDefault();

            if(((gamepad2.a || (intake.stateTransfer== Intake.TransferLogic.ReadyToTransfer && intake.extendo.state== Extendo.State.IN)) && !intake.justColorAllaiance) && outtake.state== Outtake.State.Deafult && outtake.inPosition())outtake.grabSample();

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

            if(state==State.CLIMB)
            {
                climbState.run();

                if(climbState.transition())climbState=climbState.next[Math.min(climbState.index++ , climbState.next.length-1)];
            }

            if(gamepad1.y && prevB)
            {
                state=State.SCORING_SPECIMEN;
                Odo.reset();
            }
            prevB=gamepad1.y;

            if(state==State.SCORING_SPECIMEN)
            {
                driveTrain.setMode(MecanumDriveTrain.State.PID);
                scoringSpecimenState.run();
                if(scoringSpecimenState.transition())scoringSpecimenState=scoringSpecimenState.next[Math.min(scoringSpecimenState.index++ , scoringSpecimenState.next.length-1)];

            }


            driveTrain.update();
            outtake.update();
            intake.update();
            Odo.update();
            pto.update();
            wheelie.update();

            telemetry.addData("SpeedLiftChamber" , Outtake.sumChamber/Outtake.nrChamber);
            telemetry.addData("SpeedLiftBasket" , Outtake.sumBasket/Outtake.nrBasket);

            telemetry.addData("IntakeState" , intake.state);
            telemetry.addData("claw" , outtake.claw.state.name);
            telemetry.addData("arm" , outtake.arm.state.name);
            telemetry.addData("heading" , Odo.getHeading());
            telemetry.addData("IMUOFFSET" , Hardware.IMUOFFSET);
            telemetry.addData("motor1" , Differential.motor1.motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("motor2" , Differential.motor2.motor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("IMU" , Odo.getHeading());
            telemetry.addData("TransferState" , intake.stateTransfer);
            telemetry.addData("asure1" , intake.asure1inDepositState);
            telemetry.addData("cimb" , climbState.name);
            telemetry.addData("outtakeState" , outtake.state);
            telemetry.addData("extension" , outtake.extension.state.name);
            telemetry.addData("TimerClaw" , outtake.claw.servos[0].timer.seconds());
            telemetry.addData("OdoState" , Odo.odo.getDeviceStatus());
            telemetry.addData("LoopTime" , Odo.odo.getLoopTime());

            telemetry.update();
        }
    }
}

