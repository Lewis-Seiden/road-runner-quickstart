package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

public class Voltage {
    public static double getVoltage(HardwareMap hardwareMap){
        double voltage = Double.POSITIVE_INFINITY;
        for (VoltageSensor v : hardwareMap.voltageSensor){
            double volt = v.getVoltage();
            if (volt > 0){
                voltage = Math.min(volt, voltage);
            }
        }
        return voltage;
    }
}
