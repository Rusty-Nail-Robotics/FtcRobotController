package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Show_Off {
    RevBlinkinLedDriver blinkinLedDriver;           //create container holding the led driver info
    RevBlinkinLedDriver.BlinkinPattern pattern;     //create container holding the led driver patterns


    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode){

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledServo");  //Associate in-program name with robot configuration name

    }

    void SetLedPattern(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        switch(Global_Variables.ledMode){
            case 1:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);    //Set and display emergency pattern on led driver
                break;

            case 2:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);    //Set and display gripper closed pattern on led driver
                break;

            case 3:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);    //Set and display Gripper open pattern on led driver
                break;
        }

    }
}