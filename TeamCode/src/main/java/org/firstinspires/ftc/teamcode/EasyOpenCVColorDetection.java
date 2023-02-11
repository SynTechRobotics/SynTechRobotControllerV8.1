package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

//THIS IS UNTESTED AND UNFINISHED
@Autonomous
public class EasyOpenCVColorDetection extends OpMode {

    OpenCvWebcam webcam1 = null;

    @Override
    public void init() {
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
    }

    @Override
    public void loop() {

    }

    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat Crop1;
        Mat Crop2;
        Mat Crop3;
        double leftavgfin;
        double middleavgfin;
        double rightavgfin;
        double[] avgarray = new double[3];
        Mat outPut = new Mat();
        Scalar rect1Color = new Scalar(255.0, 255.0, 0.0);
        Scalar rect2Color = new Scalar(255.0, 255.0, 0.0);
        Scalar rect3Color = new Scalar(255.0, 255.0, 0.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect crop1 = new Rect(1, 1, 425, 719);
            Rect crop2 = new Rect(426, 1, 425, 719);
            Rect crop3 = new Rect(852, 1, 425, 719);

            input.copyTo(outPut);
            Imgproc.rectangle(outPut, crop1, rect1Color, 2);
            Imgproc.rectangle(outPut, crop2, rect2Color, 2);
            Imgproc.rectangle(outPut, crop3, rect3Color, 2);

            Crop1 = YCbCr.submat(crop1);
            Crop2 = YCbCr.submat(crop2);
            Crop3 = YCbCr.submat(crop3);

            Core.extractChannel(Crop1, Crop1, 1);
            Core.extractChannel(Crop2, Crop2, 1);
            Core.extractChannel(Crop3, Crop3, 1);

            Scalar leftavg = Core.mean(Crop1);
            Scalar midavg = Core.mean(Crop2);
            Scalar rightavg = Core.mean(Crop3);

            avgarray = new double[]{leftavg.val[0], midavg.val[0], rightavg.val[0]};


            if (avgarray[0] < avgarray[1] && avgarray[0] < avgarray[2]) {
                telemetry.addLine("Left");
                rect1Color = new Scalar(0.0, 0.0, 255.0);
                rect2Color = new Scalar(255.0, 255.0, 0.0);
                rect3Color = new Scalar(255.0, 255.0, 0.0);
            } else if (avgarray[1] < avgarray[0] && avgarray[1] < avgarray[2]){
                telemetry.addLine("Middle");
                rect1Color = new Scalar(255.0, 255.0, 0.0);
                rect2Color = new Scalar(0.0, 0.0, 255.0);
                rect3Color = new Scalar(255.0, 255.0, 0.0);
            } else if (avgarray[2] < avgarray[1] && avgarray[2] < avgarray[0]) {
                telemetry.addLine("Right");
                rect1Color = new Scalar(255.0, 255.0, 0.0);
                rect2Color = new Scalar(255.0, 255.0, 0.0);
                rect3Color = new Scalar(0.0, 0.0, 255.0);
            }
            Imgproc.rectangle(outPut, crop1, rect1Color, 2);
            Imgproc.rectangle(outPut, crop2, rect2Color, 2);
            Imgproc.rectangle(outPut, crop3, rect3Color, 2);

            return(outPut);
        }
    }
}
