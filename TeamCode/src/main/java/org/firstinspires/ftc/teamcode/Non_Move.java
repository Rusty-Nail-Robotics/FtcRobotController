package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Non_Move {
    public DcMotorEx liftMotor;
    public Servo gripServo;
    public DcMotorEx londonMotor;
    public int liftTarget = 0;
    public int liftVelocity = 300;
    public int londonTarget = 0;

    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        //Lift Motor Setup (ViperSlide)
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");          //Match in-program name to item name in robot configuration
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);             //Set Motor Off behavior
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                     //Reset Encoder to zero
        liftMotor.setPower(1);                                                         //Set Maximum power
        liftMotor.setTargetPosition(liftTarget);                                       //Set the target position (required before Run_To_Position)
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                          //Set motor to run to position (Target)


        londonMotor = hardwareMap.get(DcMotorEx.class, "londonMotor");      //Match in-program name to item name in robot configuration
        londonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);           //Set Motor Off behavior
        londonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                   //Reset Encoder to zero
        londonMotor.setPower(1);                                                       //Set Maximum power
        londonMotor.setTargetPosition(londonTarget);                                   //Set the target position (required before Run_To_Position)
        londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                        //Set motor to run to position (Target)


        //gripServo = hardwareMap.get(Servo.class, "GripServo");                        //Match in-program name to item name in robot configuration
        //gripServo.setPosition(0);                                                     //Set to starting Position







    }

   void Operations(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        double liftPower = linearOpMode.gamepad1.left_trigger + -linearOpMode.gamepad1.right_trigger;   //math to make triggers generate [-1 - 1] value
       if(liftPower != 0) {                                                        //Check if value is not zero
           liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
           liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
           liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
       }else {                                                                     //If above value is 0 (no gamepad input)
           liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                   //Set motor to run to target position
           liftMotor.setPower(1);                                                  //Allow full power
       }

    if(linearOpMode.gamepad1.left_stick_y != 0) {                                   //Check if value is not zero (gamepad input)
        londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
        londonMotor.setVelocity(linearOpMode.gamepad1.left_stick_y*150);            //Apply motor velocity to match trigger inputs
        londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
    }else {                                                                         //If above value is 0 (no gamepad input)
        londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                     //Set motor to run to target position
        londonMotor.setPower(1);                                                    //Allow full power
    }
}
}
