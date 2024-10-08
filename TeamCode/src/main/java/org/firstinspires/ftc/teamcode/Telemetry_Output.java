package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Telemetry_Output {

    public void Add_Drive_Telem(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode, Telop_Chassis_Control chassisControl){
        linearOpMode.telemetry.addData("X Call",chassisControl.x);
        linearOpMode.telemetry.addData("Y Call",chassisControl.y);
        linearOpMode.telemetry.addData("R-X Call",chassisControl.rx);
        linearOpMode.telemetry.addData("gamepad 1 RightStick x = ", linearOpMode.gamepad1.right_stick_x);
        linearOpMode.telemetry.addData("gamepad 1 RightStick y = ", linearOpMode.gamepad1.right_stick_y);


    }

    public void Add_Odometry_Output(HardwareMap ignoredhardwareMap, LinearOpMode linearOpMode, double[] currentPosition){
        linearOpMode.telemetry.addData("X location = ", currentPosition[0]);
        linearOpMode.telemetry.addData("Y Location = ", currentPosition[1]);
        linearOpMode.telemetry.addData("Heading = ", currentPosition[2]);

    }
}
