
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "OpenCV Testing")

public class opencv extends LinearOpMode {

    double cX = 0;
    double cY = 0;
    double width = 0;

    private OpenCvCamera controlHubCam;  // Use OpenCvCamera class from FTC SDK
    private static final int CAMERA_WIDTH = 1280; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 720; // height of wanted camera resolution

    // Calculate the distance using the formula
    public static final double objectWidthInRealWorldUnits = 3.75;  // Replace with the actual width of the object in real-world units
    public static final double focalLength = 728;  // Replace with the focal length of the camera in pixels

    private static final double MIN_AREA_DETECTION = 2000;

    private enum SampleColor {
        RED,
        BLUE,
        YELLOW
    }

    @Override
    public void runOpMode() {

        initOpenCV();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(controlHubCam, 30);


        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) cX + ", " + (int) cY + ")");
            telemetry.addData("Distance in Inch", (getDistance(width)));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        controlHubCam.stopStreaming();
    }

    private void initOpenCV() {

        // Create an instance of the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        controlHubCam.setPipeline(new SampleDetectionPipeline());

        controlHubCam.openCameraDevice();
        controlHubCam.startStreaming(CAMERA_WIDTH, CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }
    class SampleDetectionPipeline extends OpenCvPipeline {
        @Override
        public Mat processFrame(Mat input) {
            // Preprocess the frame to detect red regions
            Mat redMask = preprocessFrame(input, SampleColor.RED);
            drawLabels(input, redMask, SampleColor.RED);

//            Mat blueMask = preprocessFrame(input, SampleColor.BLUE);
//            drawLabels(input, blueMask, SampleColor.BLUE);
//
//            Mat yellowMask = preprocessFrame(input, SampleColor.YELLOW);
//            drawLabels(input, yellowMask, SampleColor.YELLOW);

            return input;
        }

        private void drawLabels(Mat input, Mat mask, SampleColor color) {
            // Find contours of the detected red regions
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            for (MatOfPoint contour : contours) {
                labelContour(input, contour, contours, color);
            }
        }

        private void labelContour(Mat input, MatOfPoint contour, List<MatOfPoint> contours, SampleColor color) {
            double area = Imgproc.contourArea(contour);

            if (area > MIN_AREA_DETECTION) {
                // Draw a green outline around the detected object
                Imgproc.drawContours(input, contours, contours.indexOf(contour), new Scalar(0, 255, 0), 2);
                // Calculate the width of the bounding box
                width = calculateWidth(contour);

                // Display the width next to the label
                String widthLabel = "Width: " + (int) width + " pixels";
                Imgproc.putText(input, widthLabel, new Point(cX + 10, cY + 20), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                //Display the color
                String colorLabel = "Color: " + (color.toString());
                Imgproc.putText(input, colorLabel, new Point(cX + 10, cY + 40), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);

                //Display the Distance
                String distanceLabel = "Distance: " + String.format("%.2f", getDistance(width)) + " inches";
                Imgproc.putText(input, distanceLabel, new Point(cX + 10, cY + 60), Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                // Calculate the centroid of the largest contour
                Moments moments = Imgproc.moments(contour);
                cX = moments.get_m10() / moments.get_m00();
                cY = moments.get_m01() / moments.get_m00();

                // Draw a dot at the centroid
                String label = "(" + (int) cX + ", " + (int) cY + ")";
                Imgproc.putText(input, label, new Point(cX + 10, cY), Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cX, cY), 5, new Scalar(0, 255, 0), -1);
            }
        }

        private Mat preprocessFrame(Mat frame, SampleColor color) {
            Mat hsvMat = new Mat();
            Mat lowMat = new Mat();
            Mat upperMat = new Mat();
            Mat detectedMat = new Mat();

            

            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
            Scalar lowerLow = new Scalar(0);
            Scalar lowerHigh = new Scalar(0);
            Scalar upperLow = new Scalar(0);
            Scalar upperHigh = new Scalar(0);


            switch (color) {
                case RED:

                    // 0-20
                    // 330-360
                    // 50%, 50%

                    lowerLow = new Scalar(0, 125, 125);
                    lowerHigh = new Scalar(10, 255, 255);
                    upperLow = new Scalar(165, 125, 125);
                    upperHigh = new Scalar(180, 255, 255);
                    break;
                case BLUE:

                    // 50%, 50%
                    // 224-244

                    lowerLow = new Scalar(112, 125, 25);
                    lowerHigh = new Scalar(122, 255, 255);
                    upperLow = new Scalar(112, 125, 125);
                    upperHigh = new Scalar(122, 255, 255);
                    break;
                case YELLOW:
                    //44-64
                    // 50%, 50%

                    lowerLow = new Scalar(22, 125, 125);
                    lowerHigh = new Scalar(34, 255, 255);
                    upperLow = new Scalar(22, 125, 125);
                    upperHigh = new Scalar(34, 255, 255);
                    break;
            }

            Mat mask = new Mat();
            Core.inRange(hsvMat, lowerLow, lowerHigh, lowMat);
            Core.inRange(hsvMat, upperLow, upperHigh, upperMat);

            Core.bitwise_or(lowMat, upperMat, detectedMat);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            return detectedMat;
        }

        private MatOfPoint findLargestContour(List<MatOfPoint> contours) {
            double maxArea = 0;
            MatOfPoint largestContour = null;

            for (MatOfPoint contour : contours) {
                double area = Imgproc.contourArea(contour);
                if (area > maxArea) {
                    maxArea = area;
                    largestContour = contour;
                }
            }

            return largestContour;
        }
        private double calculateWidth(MatOfPoint contour) {
            Rect boundingRect = Imgproc.boundingRect(contour);
            return boundingRect.width;
        }

    }
    private static double getDistance(double width){
        double distance = (objectWidthInRealWorldUnits * focalLength) / width;
        return distance;
    }


}
