package org.firstinspires.ftc.teamcode.OpModes;


import com.acmerobotics.dashboard.FtcDashboard;
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
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Robot.Node;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "a")
public class TeleOpWithSensors_RED extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        //if(Odo.INIT)
        //Odo.odo.setPosition(new Pose2D(DistanceUnit.MM , 0 ,0 , AngleUnit.RADIANS , Odo.getHeading()-Hardware.IMUOFFSET));

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Node currentNode;


        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Intake intake=new Intake(SampleColor.State.RED , true);

        PTO pto=new PTO();
        Wheelie wheelie=new Wheelie();

        outtake.extension.setState("goRetrect");
        outtake.arm.setState("goHighSpecimen");

        while(opModeInInit())
        {
            outtake.extension.update();
            outtake.arm.update();
            intake.update();
            Hardware.ssh3.setPosition(0.34);
        }
        waitForStart();

        while(opModeIsActive()){

            double X=gamepad1.left_stick_x * gamepad1.left_stick_x * Math.signum(gamepad1.left_stick_x);
            double Y=gamepad1.left_stick_y * gamepad1.left_stick_y * -Math.signum(gamepad1.left_stick_y);
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            double heading =-Odo.getHeading();

            double x=X*Math.cos(heading)-Y*Math.sin(heading);
            double y=X* Math.sin(heading)+Y*Math.cos(heading);

            driveTrain.setTargetVector( x , y , rotation );

            if(gamepad1.options) Odo.reset();




            gamepad1.setLedColor(44, 128, 14 , 10000);
            gamepad2.setLedColor(163, 139, 191, 10000);

            if(gamepad1.a)intake.setExtendoIN();

            if(gamepad2.y && outtake.state!=Outtake.State.TakingElement)outtake.goDefault();

            if(((gamepad2.a || (intake.state==Intake.State.TRANSFER && intake.ramp.state==intake.ramp.states.get("up") && intake.extendo.state== Extendo.State.IN && intake.kNR>=2)) && !intake.justColorAllaiance) && outtake.state== Outtake.State.Deafult && outtake.inPosition())outtake.grabSample();

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

            driveTrain.update();
            wheelie.update();
            outtake.update();
            intake.update();
            Odo.update();
            pto.update();

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
            telemetry.update();
        }
    }
}

