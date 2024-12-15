package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);             //Set Motor Off behavior
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                     //Reset Encoder to zero
        liftMotor.setPower(1);                                                         //Set Maximum power
        liftMotor.setTargetPosition(liftTarget);                                       //Set the target position (required before Run_To_Position)
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                          //Set motor to run to position (Target)


        londonMotor = hardwareMap.get(DcMotorEx.class, "londonMotor");      //Match in-program name to item name in robot configuration
        londonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);           //Set Motor Off behavior
        londonMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        londonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);                   //Reset Encoder to zero
        londonMotor.setPower(1);                                                       //Set Maximum power
        londonMotor.setTargetPosition(londonTarget);                                   //Set the target position (required before Run_To_Position)
        londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                        //Set motor to run to position (Target)


        gripServo = hardwareMap.get(Servo.class, "GripServo");                        //Match in-program name to item name in robot configuration
        gripServo.setPosition(Global_Variables.gripperOpen);                                                     //Set to starting Position


    }

    void LiftOperations(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode) {
        if(liftMotor.isOverCurrent()){
            liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            liftMotor.setPower(0);
            Global_Variables.ledMode = 1;
        }

        double liftPower = linearOpMode.gamepad1.right_trigger - linearOpMode.gamepad1.left_trigger;   //math to make triggers generate [-1 - 1] value

        if (liftPower != 0) {                                                        //Check if value is not zero
            if(liftMotor.getCurrentPosition() < Global_Variables.viperMax && liftPower > 0) {
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            }
            else if(liftMotor.getCurrentPosition() > Global_Variables.viperMin && liftPower < 0){
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now

            }
            else{
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(0);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            }
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
        } else {                                                                     //If above value is 0 (no gamepad input)
            if(liftMotor.getCurrentPosition() > Global_Variables.viperMax){
                liftMotor.setTargetPosition(Global_Variables.viperMax);
            }
            if(liftMotor.getCurrentPosition() < Global_Variables.viperMin){
                liftMotor.setTargetPosition(Global_Variables.viperMin);
            }
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                   //Set motor to run to target position
            liftMotor.setPower(1);                                                  //Allow full power
        }


        double londonInput = linearOpMode.gamepad1.left_stick_y;
        int londonMotorPos = londonMotor.getCurrentPosition();
        double londonMotorOutput = 0;
        double lastLondonMotorOutput = 0;


        linearOpMode.telemetry.addData("Current London = ", londonMotorPos);
        linearOpMode.telemetry.addData("London Cont Input = ", londonInput);
        linearOpMode.telemetry.addData("Viper Position = ", liftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("Viper Cont Input = ", liftPower);
        linearOpMode.telemetry.addData("Limit Check = ", CheckLondonLimit(londonMotorPos));
        linearOpMode.telemetry.addData("London Min =  ", map(liftMotor.getCurrentPosition(), Global_Variables.viperMin, Global_Variables.viperMax, Global_Variables.londonMinAtRetraction, Global_Variables.londonMinAtExtention));
        linearOpMode.telemetry.addData("London Max =  ",map(liftMotor.getCurrentPosition(), Global_Variables.viperMin, Global_Variables.viperMax, Global_Variables.londonMaxAtRetraction, Global_Variables.londonMaxAtExtention));
        if(linearOpMode.gamepad1.left_bumper){
            londonMotor.setTargetPosition(Global_Variables.londonLiftTarget);
            londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                     //Set motor to run to target position
            londonMotor.setPower(1);                                                    //Allow full power

            //londonMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //londonMotor.setPower(0);
            //londonMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if(!linearOpMode.gamepad1.left_bumper){
        if (londonInput != 0) {                                   //Check if value is not zero (gamepad input)
            if(CheckLondonLimit(londonMotorPos) != 1 && londonInput > 0) {
                londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                londonMotorOutput = (linearOpMode.gamepad1.left_stick_y * Global_Variables.londonLiftGain);            //Apply motor velocity to match trigger inputs

                //londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            } else if (CheckLondonLimit(londonMotorPos) != -1 && londonInput < 0) {
                londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                londonMotorOutput = (linearOpMode.gamepad1.left_stick_y * Global_Variables.londonLowerGain);            //Apply motor velocity to match trigger inputs
                //londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            }else{
                londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                londonMotor.setVelocity(0);            //Apply motor velocity to match trigger inputs
                londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now

            }
            if(lastLondonMotorOutput != londonMotorOutput){
                londonMotor.setVelocity(londonMotorOutput);
                //lastLondonMotorOutput = londonMotorOutput;
            }
            londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
        } else {                                                                         //If above value is 0 (no gamepad input)
            if(CheckLondonLimit(londonMotorPos) == 1){
                londonMotor.setTargetPosition(Global_Variables.londonMaxAtExtention);
            }
            if(CheckLondonLimit(londonMotorPos) == -1){
                londonMotor.setTargetPosition(Global_Variables.londonMinAtExtention);
            }
            londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                     //Set motor to run to target position
            londonMotor.setPower(1);                                                    //Allow full power
        }
    }

    }

    int CheckLondonLimit(int currentPos){
        int londonMin = Global_Variables.londonMinAtExtention;//(int)map(liftMotor.getCurrentPosition(), Global_Variables.viperMin, Global_Variables.viperMax, Global_Variables.londonMinAtRetraction, Global_Variables.londonMinAtExtention);
        int londonMax = Global_Variables.londonMaxAtExtention; //(int)map(liftMotor.getCurrentPosition(), Global_Variables.viperMin, Global_Variables.viperMax, Global_Variables.londonMaxAtRetraction, Global_Variables.londonMaxAtExtention);

        if(currentPos < londonMin){
            return -1;
        }
        if(currentPos > londonMax){
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
            }
            } else {                                                                             //if x is released
                xPressed = 0;                                                                   //Remember that the code needs ran on the next x press


            }


    }

    double map(double x, double in_min, double in_max, double out_min, double out_max)
    {
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    }
}
