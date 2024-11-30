package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class BbTest extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DigitalChannel bb;
        bb=hardwareMap.get(DigitalChannel.class , "bb");

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData( "Sebi e fraier" , bb.getState());
            telemetry.update();
        }
    }
}
