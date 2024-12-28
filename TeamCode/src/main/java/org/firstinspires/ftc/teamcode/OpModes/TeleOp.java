package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Climb.Climb;
import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Lift;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "a")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware.init(hardwareMap);
        Odo.init(hardwareMap , telemetry);

        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Extendo extendo=new Extendo();
        Intake intake=new Intake();
        Latch latch=new Latch();
       Climb climb=new Climb();

        boolean a=false;
        DigitalChannel bb;

        while(opModeInInit())
        {
            outtake.update();
            intake.update();
            extendo.update();
            driveTrain.update();
            latch.update();
        }
        waitForStart();

        while(opModeIsActive()){

            double x=gamepad1.left_stick_x;
            double y=-gamepad1.left_stick_y;
            double rotation=gamepad1.right_trigger-gamepad1.left_trigger;

            if(gamepad1.a && gamepad1.dpad_down)a=true;
            if(gamepad1.b && gamepad1.dpad_down) a=false;

            if(a==false){
                if(gamepad1.dpad_up) climb.goUp();
                else if(gamepad1.dpad_down)climb.goDown();
                else climb.Default();}
            else climb.Constant();

            driveTrain.setTargetVector( x , y , rotation );

            double power=-gamepad1.right_stick_y;
            extendo.setVelocity(power);
            if(gamepad1.a)extendo.setIn();


            if(gamepad1.options) Odo.reset();

            if(gamepad2.y)outtake.goDefault();

            if(gamepad2.a && outtake.state== Outtake.State.Deafult)
            {
                latch.setState("goOpen");
            }
            if(latch.state==latch.states.get("open"))
            {
                outtake.grabSample();
            }
           if(latch.state==latch.states.get("open") && outtake.arm.state!=outtake.arm.states.get("deposit") && outtake.arm.state!=outtake.arm.states.get("goWithElement") && outtake.arm.state!=outtake.arm.states.get("goDespoit") && outtake.arm.state!=outtake.arm.states.get("takeElement"))
                latch.setState("goClose");

            if(gamepad2.dpad_up && outtake.state==Outtake.State.DeafultWithElement)outtake.goUp();

            if(gamepad1.circle && outtake.state==Outtake.State.DeafultWithElement)
            {
                outtake.releaseSample();
            }

            if(gamepad1.circle && outtake.state==Outtake.State.Specimen)outtake.grabSample();

            if(gamepad1.circle){
                if(outtake.haveSample && outtake.state== Outtake.State.Up)
                outtake.lift.setPosition(Lift.position-10);
                outtake.score();}

            if(gamepad2.dpad_left)outtake.goForLow();
            if(gamepad2.dpad_right)outtake.goForHigh();

            if(gamepad2.circle && outtake.state== Outtake.State.Deafult)
            {
                outtake.takeSpecimen();
            }

            if(gamepad2.right_trigger>0)intake.setState(Intake.State.INTAKE_DOWN);
            else if(gamepad2.left_trigger>0)intake.setState(Intake.State.REVERSE_DOWN);
            else if(extendo.state== Extendo.State.GOING_IN || Math.abs(power)>0.1)intake.setState(Intake.State.INTAKE_UP);
            else intake.setState(Intake.State.REPAUS_UP);







            telemetry.addData("extendo" , extendo.state);
            telemetry.addData("outtake" , outtake.state);
            telemetry.addData("intake" , intake.state);
            telemetry.addData("latch" , latch.state.name);
            telemetry.addData("claw" , outtake.claw.state.name);
            telemetry.addData("haveSample" , outtake.haveSample);
            telemetry.addData("heading",Odo.getHeading());
         //   telemetry.addData("liftPosition" , outtake.lift.motorLeft.getPosition());
            telemetry.addData("latch" , latch.state.name);
            telemetry.addData("arm" , outtake.arm.state.name);



            telemetry.update();
            driveTrain.update();
            outtake.update();
            extendo.update();
            intake.update();
            latch.update();
            Odo.update();
        }
    }
}