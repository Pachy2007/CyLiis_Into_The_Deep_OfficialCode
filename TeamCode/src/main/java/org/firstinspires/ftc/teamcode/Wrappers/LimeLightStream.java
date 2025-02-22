package org.firstinspires.ftc.teamcode.Wrappers;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.function.ContinuationResult;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.opencv.android.Utils;
import org.opencv.core.CvException;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.opencv.videoio.VideoCapture;

public class LimeLightStream implements CameraStreamSource {
    VideoCapture capture;
    public LimeLightStream(){
        capture = new VideoCapture("http://172.29.0.1:5800/stream.mjpg");
    }
    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation) {
        Mat frame = new Mat();
        capture.read(frame);
        continuation.dispatch((ContinuationResult<Consumer<Bitmap>>) consumer -> consumer.accept(convertMatToBitMap(frame)));
    }

    public Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = null;
        Mat rgb = input;
//        Imgproc.cvtColor(input,rgb,Imgproc.COLOR_BGR2RGB);
        try{
            bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(),Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgb,bmp);
        }catch (CvException ignored){
            throw ignored;
        }
        return bmp;
    }
}
