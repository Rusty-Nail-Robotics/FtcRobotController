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
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);    //Set and display pattern on led driver
    }
}