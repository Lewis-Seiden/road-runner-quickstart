package org.firstinspires.ftc.teamcode.vision;


import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class UltamiteGoalPipeline extends OpenCvPipeline {
    public double threshold;
    public double x1BottomRectangle = 155;
    public double y1BottomRectangle = 11;
    public double x2BottomRectangle = 159;
    public double y2BottomRectangle = 39;
    /*coordinates that check for the bottom ring.
    You can find the values by running VisionTester.
    x1 is what x is, x2 is x plus width, y1 is what y is y2 is y plus height.
    Round down when adding
     */
    public double x1TopRectangle = 137;
    public double y1TopRectangle = 23;
    public double x2TopRectangle = 156;
    public double y2TopRectangle = 45;
    //coordinates of the box that checks for the top three rings. Find coordinates in same way as bottom rectangle
    public double thresholdLow = 7;
    public double thresholdHigh = 16;
    //the two values that the hsv mean of the rings fall between. You can see the hsv mean by running vision tester.
    public int stack;
    public boolean checkThreshold(double checkThreshold) {
        return checkThreshold > thresholdLow && checkThreshold < thresholdHigh;
    }
    public Mat processFrame (Mat input) {
        Mat convertedMat = new Mat();
        Imgproc.cvtColor(input, convertedMat, Imgproc.COLOR_RGB2HSV);
        Scalar meanTopRect = Core.mean(convertedMat.submat(new Rect(new Point(x1TopRectangle, y1TopRectangle), new Point(x2TopRectangle, y2TopRectangle))));
        Scalar meanBottomRect = Core.mean(convertedMat.submat(new Rect(new Point(x1BottomRectangle, y1BottomRectangle), new Point(x2BottomRectangle, y2BottomRectangle))));
        // gets average hsv value for rectangle
        if(checkThreshold(meanTopRect.val[0])){
            stack = 4;
        }else if(checkThreshold(meanBottomRect.val[0])) {
            stack = 1;
        }else{
            stack = 0;
        }
        Imgproc.cvtColor(input, convertedMat, Imgproc.COLOR_RGB2YCrCb);
        return input;

    }
}
