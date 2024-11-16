package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Show_Off {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public Servo ledServo;


    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode){

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledServo");




    }

    void SetLedPattern(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
    }
}