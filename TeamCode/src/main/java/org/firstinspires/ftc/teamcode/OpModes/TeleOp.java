package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Modules.Drive.MecanumDriveTrain;
import org.firstinspires.ftc.teamcode.Modules.Intake.Extendo;
import org.firstinspires.ftc.teamcode.Modules.Intake.Intake;
import org.firstinspires.ftc.teamcode.Modules.Others.Latch;
import org.firstinspires.ftc.teamcode.Modules.Outtake.Outtake;
import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Imu;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "Z")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Hardware.init(hardwareMap);
        Imu.init(hardwareMap);

        Outtake outtake=new Outtake();
        MecanumDriveTrain driveTrain=new MecanumDriveTrain(MecanumDriveTrain.State.DRIVE);
        Extendo extendo=new Extendo();
        Intake intake=new Intake();
        Latch latch=new Latch();

        ColorRangeSensor sensor;
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

            driveTrain.setPower( x , y , rotation );

            double power=-gamepad1.right_stick_y;
            extendo.setVelocity(power);
            if(gamepad1.a)extendo.setIn();


            if(gamepad1.options) Imu.reset();

            if(gamepad2.y)outtake.goDefault();

            if(gamepad2.a && outtake.state== Outtake.State.Deafult)
            {outtake.grabSample();
                latch.setState("goOpen");
            }
            if(latch.state==latch.states.get("open"))
            {
                outtake.grabSample();
            }
            if(latch.state==latch.states.get("open") && outtake.state!=Outtake.State.TakingElement && outtake.arm.state!=outtake.arm.states.get("goWithElement"))
                latch.setState("goClose");

            if(gamepad2.dpad_up && outtake.state==Outtake.State.DeafultWithElement)outtake.goUp();

            if(gamepad1.circle && outtake.state==Outtake.State.DeafultWithElement)
            {
                outtake.releaseSample();
            }

            if(gamepad1.circle)outtake.score();

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
            telemetry.addData("heading",driveTrain.getHeading());



            telemetry.update();
            driveTrain.update();
            outtake.update();
            extendo.update();
            intake.update();
            latch.update();
            Imu.update();
        }
    }
}