package org.firstinspires.ftc.teamcode.OpModes;

import android.provider.MediaStore;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamClient;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.Wrappers.LimeLightStream;
import org.opencv.videoio.VideoCapture;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(group = "zzzz")
public class DashboardCamera extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Limelight3A limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        waitForStart();

        LimeLightStream stream = new LimeLightStream();
        FtcDashboard.getInstance().startCameraStream(stream,30);
        while(opModeIsActive()){

        }
    }
}
