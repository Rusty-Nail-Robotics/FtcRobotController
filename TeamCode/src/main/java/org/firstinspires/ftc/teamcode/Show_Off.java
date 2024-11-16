package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.Servo;

public class Show_Off {

    public Servo ledServo;


    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode){

        ledServo = hardwareMap.get(Servo.class, "ledServo");




    }

    void Operations(HardwareMap hardwareMap, LinearOpMode linearOpMode){

    }
}