package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveToTarget {

    public double[] Drive_Calculation(LinearOpMode linearOpMode, double targetX, double targetY, double targetRX, double[] currentPosition){
    double xToTarget = currentPosition[0] - targetX;
    double yToTarget = currentPosition[1] - targetY;
    double rxToTarget = currentPosition[2] - targetRX;


    //double x = 2;
    //double y = 2;
    //double rx = 2;
    double[] toTarget = {xToTarget,yToTarget,rxToTarget};
    return toTarget;
    }
}
