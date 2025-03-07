package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp
public class HubImu extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        List<LynxModule> h = hardwareMap.getAll(LynxModule.class);



        waitForStart();
        while(opModeIsActive()){
            for(LynxModule e : h){
                telemetry.addData("module " + e.isParent(), e.getImuType().toString());
            }
            telemetry.update();
        }
    }
}