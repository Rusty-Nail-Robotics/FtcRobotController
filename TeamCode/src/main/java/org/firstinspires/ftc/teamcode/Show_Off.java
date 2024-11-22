package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;



public class Show_Off {
    RevBlinkinLedDriver blinkinLedDriver;           //create container holding the led driver info
    //RevBlinkinLedDriver.BlinkinPattern pattern;     //create container holding the led driver patterns


    void Setup(HardwareMap hardwareMap, LinearOpMode ignoredLinearOpMode){

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledServo");  //Associate in-program name with robot configuration name

    }

    void SetLedPattern(HardwareMap ignoredHardwareMap, LinearOpMode ignoredLinearOpMode){
        switch(Global_Variables.ledMode){
            case 0:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.TWINKLES_RAINBOW_PALETTE);    //Set and display emergency pattern on led driver
                break;
            case 1:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);    //Set and display emergency pattern on led driver
                break;

            case 2:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);    //Set and display gripper closed pattern on led driver
                break;

            case 3:
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);    //Set and display Gripper open pattern on led driver
                break;
        }

    }
}