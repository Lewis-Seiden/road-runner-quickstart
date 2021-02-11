package org.firstinspires.ftc.teamcode.Odometry;

public class Odometry {

    Encoder leftEncoder;
    Encoder rightEncoder;
    Encoder centerEncoder;
    double xPosition;
    double yPosition;
    double heading;
    double previousLeftEncoderReading;
    double previousRightEncoderReading;
    double previousCenterEncoderReading;
    boolean firstLoopCall;
    
    public Odometry(Encoder leftEncoder, Encoder rightEncoder, Encoder centerEncoder) {
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.centerEncoder = centerEncoder;
        firstLoopCall = true;
    }

    public void loop(){
        if (firstLoopCall) {
            previousCenterEncoderReading = centerEncoder.getEncoderPosition();
            previousLeftEncoderReading = leftEncoder.getEncoderPosition();
            previousRightEncoderReading = rightEncoder.getEncoderPosition();
            xPosition = 0;
            yPosition = 0;
            heading = 0;
        }
    }
}
