package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class Camera
{
    private static Pipeline pipeline;
    private static OpenCvWebcam phoneCam;

    public static void CamInit(HardwareMap hwMap)
    {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class,"Webcam 1"), cameraMonitorViewId);
        pipeline = new Pipeline();
        phoneCam.setPipeline(pipeline);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                phoneCam.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }
        });
    }
    public static class Pipeline extends OpenCvPipeline
    {
        static final Scalar ORANGE_MIN = new Scalar(12, 80, 40);
        static final Scalar ORANGE_MAX = new Scalar(18, 255, 255);
        Rect rect;
        public double count = 0;
        Mat element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,new Size(25, 5));

        @Override
        public Mat processFrame(Mat input)
        {
            Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
            Imgproc.GaussianBlur(input, input, new  org.opencv.core.Size(5, 5), 5);
            Core.inRange(input, ORANGE_MIN, ORANGE_MAX, input);
            Imgproc.erode(input, input, element);
            Imgproc.dilate(input, input, element);
            rect = Imgproc.boundingRect(input);
            if (rect.height != 0.0 && rect.width != 0.0)  count = (double) rect.height / rect.width;
            else count = -1.0;
            return input;
        }
        public double GetCircles(){
            return count;
        }
    }
   public static int GetCircles()
    {/*
        return pipeline.count;
    }*/
        if (pipeline.count > 0.2){
            phoneCam.closeCameraDevice();
            return 4;
        }
        else if (pipeline.count > 0){
            phoneCam.closeCameraDevice();
            return 1;
        }
        phoneCam.closeCameraDevice();
        return 0;
    }
    public static void StopCam() {
        phoneCam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {

            }
        });
    }
}