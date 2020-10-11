package org.firstinspires.ftc.team417_2020;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@TeleOp(name = "Test Detection")
public class KaitlinTestOpenCVDetection extends LinearOpMode {

    OpenCvCamera webcam;
    TestPipeLine detector = new TestPipeLine();

    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the webcam to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                 * supports streaming from the webcam in the uncompressed YUV image format. This means
                 * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                 * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                 *
                 * Also, we specify the rotation that the webcam is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                webcam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
            }
        });
        webcam.setPipeline(detector);

        /*
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        phoneCam.setPipeline(new SamplePipeline() );
        phoneCam.startStreaming(640, 480, OpenCvCameraRotation.UPRIGHT);
        */

        telemetry.addLine("Waiting for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())
        {
            idle();
        }

        // close camera
        webcam.stopStreaming();
        webcam.closeCameraDevice();


    }

    static class TestPipeLine extends OpenCvPipeline {
        // store orange to Rect if area greater than ___
        // if Rect null then 0



        private Mat displayMat = new Mat();
        private Mat yuv = new Mat();
        private Mat u = new Mat();
        private Mat blurred = new Mat();
        private ArrayList<Mat> yuvSplit = new ArrayList<>();
        private ArrayList<MatOfPoint> contours = new ArrayList<>();
        private Mat hierarchy = new Mat();

        @Override
        public Mat processFrame(Mat input) {

            // copy input frame to different Mat as to not alter input
            input.copyTo(displayMat);

            // blur image
            Imgproc.GaussianBlur(input, blurred, new Size(11,11), 0);
            // convert colorspace to YUV
            Imgproc.cvtColor(input, yuv,Imgproc.COLOR_BGR2YUV);
            // split blurred YUV image into YUV channels

            Core.split(yuv, yuvSplit); // todo: fix memory leak

            /*
            u = yuvSplit.get(1);

            // filter U channel of frame
            Imgproc.threshold(u, u, 110,255, Imgproc.THRESH_BINARY_INV);

            // find contours
            Imgproc.findContours(u, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            // find largest contour and assign to variable maxRect
            double maxArea = 0.0;
            Rect rect;
            Rect maxRect = new Rect(0,0,0,0);

            for (MatOfPoint contour: contours){
                rect = Imgproc.boundingRect(contour);
                double area = Imgproc.contourArea(contour);
                if (area > maxArea && area > 9000){
                    maxArea = area;
                    maxRect = rect;
                }
            }

            // draw rectangle around largest contour
            Imgproc.rectangle(displayMat, new Point(maxRect.x, maxRect.y),
                    new Point(maxRect.x + maxRect.width, maxRect.y + maxRect.height),
                    new Scalar(0,0,0), 3);
            return displayMat;
            */
            return displayMat;
        }




    }
}
