package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

//
public class TestPipeline extends OpenCvPipeline {
    public double x;
    public double y;
    public double width;
    public double height;
    public double hsvValue;
    public TestPipeline() {
        super();
        x = 140;
        y = 100;
        width = 20;
        height = 20;
        hsvValue = 0;
    }
    public Mat processFrame (Mat input) {
        Mat hsvmat =  new Mat();
        Imgproc.cvtColor(input, hsvmat, Imgproc.COLOR_RGB2HSV);
        Scalar mean = Core.mean(hsvmat.submat(new Rect(new Point(x,y),new Point(x + width, y + height))));
        Imgproc.rectangle(input, new Rect((int)x, (int)y, (int) width, (int) height), new Scalar(255, 255, 255));
        hsvValue = mean.val[0];
        return input;
    }
}
