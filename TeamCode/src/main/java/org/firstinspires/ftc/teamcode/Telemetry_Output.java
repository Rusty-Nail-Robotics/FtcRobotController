package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Telemetry_Output(HardwareMap hardwareMap, LinearOpMode linearOpMode) {

    public void Drive_Telem(HardwareMap hardwareMap, LinearOpMode linearOpMode, Telop_Chassis_Control chassisControl){
        linearOpMode.telemetry.addData("X Call",chassisControl.x);
        linearOpMode.telemetry.addData("Y Call",chassisControl.y);
        linearOpMode.telemetry.addData("R-X Call",chassisControl.rx);
        linearOpMode.telemetry.addData("gamepad 1 RightStick x = ", linearOpMode.gamepad1.right_stick_x);
        linearOpMode.telemetry.addData("gamepad 1 RightStick y = ", linearOpMode.gamepad1.right_stick_y);
        linearOpMode.telemetry.update();

    }

    public void Odometry_Output(HardwareMap hardwareMap, LinearOpMode linearOpMode, PinPoint_Sensor pinPointSensor){

    }
}
