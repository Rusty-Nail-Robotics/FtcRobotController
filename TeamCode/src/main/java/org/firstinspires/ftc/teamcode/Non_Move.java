package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Non_Move {
    public DcMotorEx liftMotor;
    public Servo gripServo;
    public DcMotorEx londonMotor;
    public int liftTarget = 0;
    public int londonTarget = 0;

    private int xPressed = 0;

    void Setup(HardwareMap hardwareMap, LinearOpMode ignoredLinearOpMode) {
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


        gripServo = hardwareMap.get(Servo.class, "GripServo");                        //Match in-program name to item name in robot configuration
        gripServo.setPosition(0);                                                     //Set to starting Position


    }

    void LiftOperations(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode) {
        double liftPower = linearOpMode.gamepad1.left_trigger - linearOpMode.gamepad1.right_trigger;   //math to make triggers generate [-1 - 1] value
        if (liftPower != 0) {                                                        //Check if value is not zero
            liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
            liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
        } else {                                                                     //If above value is 0 (no gamepad input)
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                   //Set motor to run to target position
            liftMotor.setPower(1);                                                  //Allow full power
        }
        double londonInput = linearOpMode.gamepad1.left_stick_y;
        int londonMotorPos = londonMotor.getCurrentPosition();
        linearOpMode.telemetry.addData("Current London = ", londonMotorPos);
        linearOpMode.telemetry.addData("Viper Position = ", liftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Limit Check = ", CheckLondonLimit(londonMotorPos));
        if (londonInput != 0) {                                   //Check if value is not zero (gamepad input)
            if(CheckLondonLimit(londonMotorPos) != 1 && londonInput > 0) {
                londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                londonMotor.setVelocity(linearOpMode.gamepad1.left_stick_y * Global_Variables.londonLiftGain);            //Apply motor velocity to match trigger inputs
                londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            } else if (CheckLondonLimit(londonMotorPos) != -1 && londonInput < 0) {
                londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                londonMotor.setVelocity(linearOpMode.gamepad1.left_stick_y * Global_Variables.londonLowerGain);            //Apply motor velocity to match trigger inputs
                londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            }

        } else {                                                                         //If above value is 0 (no gamepad input)
            if(CheckLondonLimit(londonMotorPos) < 0){
                londonMotor.setTargetPosition((int)map(londonMotorPos, Global_Variables.londonMinAtRetraction, Global_Variables.londonMinAtExtention, Global_Variables.viperMin, Global_Variables.viperMax));
            }
            londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                     //Set motor to run to target position
            londonMotor.setPower(1);                                                    //Allow full power
        }
    }

    int CheckLondonLimit(int currentPos){
        if(currentPos < ( map(currentPos, Global_Variables.londonMinAtRetraction, Global_Variables.londonMinAtExtention, Global_Variables.viperMin, Global_Variables.viperMax))){
            return -1;
        }
        if(currentPos > ( map(currentPos, Global_Variables.londonMaxAtRetraction, Global_Variables.londonMaxAtExtention, Global_Variables.viperMin, Global_Variables.viperMax))){
            return 1;
        }else{
            return 0;
        }
    }

    void TelopGripperOperations(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode) {
        if (linearOpMode.gamepad1.x) {                                                       //when x is pressed
            if (xPressed == 0) {                                                              //Confirm this is the first look while pressed
                if (gripServo.getPosition() == Global_Variables.gripperOpen) {                //if the gripper is already open
                    gripServo.setPosition(Global_Variables.gripperClosed);                  //Close the gripper
                    Global_Variables.ledMode = 2;                                           //Change the lights

                } else if (gripServo.getPosition() == Global_Variables.gripperClosed) {     //if gripper was not open, but is closed
                    gripServo.setPosition(Global_Variables.gripperOpen);                    //open the gripper
                    Global_Variables.ledMode = 3;                                           //Change the lights to match
                }
                xPressed = 1;                                                               //Remember that above code has already ran

            } else {                                                                             //if x is released
                xPressed = 0;                                                                   //Remember that the code needs ran on the next x press


            }
        }

    }

    double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
