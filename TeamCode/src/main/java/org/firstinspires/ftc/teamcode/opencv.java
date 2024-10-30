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
            //make a function that filters out the other colors, so the telemetry only displays 1 color
            // Preprocess the frame to detect red regions
            Mat redMask = preprocessFrame(input, SampleColor.RED);
            drawLabels(input, redMask, SampleColor.RED);
//
            Mat blueMask = preprocessFrame(input, SampleColor.BLUE);
            drawLabels(input, blueMask, SampleColor.BLUE);

            Mat yellowMask = preprocessFrame(input, SampleColor.YELLOW);
            drawLabels(input, yellowMask, SampleColor.YELLOW);

            return input;
        }

//        private Mat redProcessFrame(Mat input) {
//            Mat redMask = preprocessFrame(input, SampleColor.RED);
//            drawLabels(input, redMask, SampleColor.RED);
//            return input;
//        }
//        private Mat blueProcessFrame(Mat input) {
//            Mat blueMask = preprocessFrame(input, SampleColor.BLUE);
//            drawLabels(input, blueMask, SampleColor.BLUE);
//            return input;
//        }
//        private Mat yellowProcessFrame(Mat input) {
//            Mat yellowMask = preprocessFrame(input, SampleColor.YELLOW);
//            drawLabels(input, yellowMask, SampleColor.YELLOW);
//            return input;
//        }
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
            Mat hsvFrame = new Mat();
            Imgproc.cvtColor(frame, hsvFrame, Imgproc.COLOR_RGB2HSV);
            Scalar lower = new Scalar(0);
            Scalar upper = new Scalar(0);

            switch (color) {
                case RED:
                    lower = new Scalar(80, 85, 100);
                    upper = new Scalar(180, 255, 255);
                    break;
                case BLUE:
//                    lower = new Scalar(90, 100, 100);
//                    upper = new Scalar(135, 215, 200);
                    lower = new Scalar(80, 85, 90);
                    upper = new Scalar(150, 215, 200);
                    break;
                case YELLOW:
                    //lower = 40, 85, 100
                    //upper = 60, 86, 79
                    lower = new Scalar(15, 50, 70);
                    upper = new Scalar(24, 255, 255);
                    break;
            }

            Mat mask = new Mat();
            Core.inRange(hsvFrame, lower, upper, mask);

            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
            Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

            return mask;
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