package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot.Hardware;
import org.firstinspires.ftc.teamcode.Wrappers.Odo;

@TeleOp(group = "x")
public class OdoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {


        Hardware.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Odo.init(hardwareMap , telemetry);

        waitForStart();


        while(opModeIsActive())
        {

            telemetry.addData("Extendo" , Hardware.mch1.getCurrentPosition());
            telemetry.addData("X" , Odo.getX());
            telemetry.addData("Y"  , Odo.getY());
            telemetry.addData("Heading" , Odo.getHeading());

            telemetry.update();
            Odo.update();

        }
    }
}
