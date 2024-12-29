package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Non_Move {
    public DcMotorEx liftMotor;
    public Servo gripServo;
    public DcMotorEx londonMotor;
    public DigitalChannel viperMinMagSwitch;
    public int liftTarget = 0;
    public int londonTarget = 0;
    private double liftPower = 0;
    private int xPressed = 0;
    private double londonInput = 0;
    private double lastLondonMotorOutput = 0;

    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode) {
        //Lift Motor Setup (ViperSlide)
        Global_Variables.basketMode = 0;
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
        viperMinMagSwitch = hardwareMap.get(DigitalChannel.class, "Viper min sensor");                        //Match in-program name to item name in robot configuration


    }

    void LiftOperations(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode) {

        linearOpMode.telemetry.addData("Viper Min Sensor = ", viperMinMagSwitch.getState());

        if(linearOpMode.gamepad1.dpad_up){
            Global_Variables.basketMode = 1;
        } else if (linearOpMode.gamepad1.dpad_down || linearOpMode.gamepad1.right_stick_button || linearOpMode.gamepad1.left_bumper) {
            Global_Variables.basketMode = 2;
        }
        Global_Variables.speedOverride = map(liftMotor.getCurrentPosition(), Global_Variables.viperMin, Global_Variables.viperMax, 1,.25);


        switch (Global_Variables.basketMode){
            case 0:
                liftPower = linearOpMode.gamepad1.right_trigger - linearOpMode.gamepad1.left_trigger;   //math to make triggers generate [-1 - 1] value
                londonInput = linearOpMode.gamepad1.left_stick_y;
                ManualLiftControl();
                ManualLondonControl();
                linearOpMode.telemetry.addData("Manual Mode", 1);
                break;

            case 1:

                    if (londonMotor.getCurrentPosition() <= Global_Variables.minBeforeExt) {
                        londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);
                        londonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        londonMotor.setPower(1);
                    } else {
                        londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);
                        londonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        londonMotor.setPower(1);
                        liftMotor.setTargetPosition(Global_Variables.basketLiftTarget);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1);
                    }
                    if (linearOpMode.gamepad1.left_stick_y != 0 || linearOpMode.gamepad1.left_trigger != 0 || linearOpMode.gamepad1.right_trigger != 0) {
                        Global_Variables.basketMode = 0;
                    }
                    linearOpMode.telemetry.addData("Basket Height Mode", 1);
                    break;

            case 2:

                    if (liftMotor.getCurrentPosition() >= Global_Variables.maxViperUnderMinBeforeLift) {
                        liftMotor.setTargetPosition(Global_Variables.fastDownLiftTarget);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1);
                    } else {
                        liftMotor.setTargetPosition(Global_Variables.fastDownLiftTarget);
                        liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        liftMotor.setPower(1);
                        londonMotor.setTargetPosition(Global_Variables.fastDownLondonTarget);
                        londonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        londonMotor.setPower(.25);
                    }
                    if (linearOpMode.gamepad1.left_stick_y != 0 || linearOpMode.gamepad1.left_trigger != 0 || linearOpMode.gamepad1.right_trigger != 0) {
                        Global_Variables.basketMode = 0;
                    }
                    linearOpMode.telemetry.addData("Fast Down Mode", 1);
                    break;


        }
        linearOpMode.telemetry.addData("Lift Pos (Viper) = ", liftMotor.getCurrentPosition());
        linearOpMode.telemetry.addData("London Pos = ", londonMotor.getCurrentPosition());
        if (londonMotor.getCurrentPosition()<Global_Variables.minBeforeExt && liftMotor.getCurrentPosition() > Global_Variables.maxViperUnderMinBeforeLift){
            liftMotor.setTargetPosition(Global_Variables.maxViperUnderMinBeforeLift);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }






    }

    void ManualLondonControl(){
        int londonMotorPos = londonMotor.getCurrentPosition();
        double londonMotorOutput = 0;
            if (londonInput != 0) {                                   //Check if value is not zero (gamepad input)
                if(CheckLondonLimit(londonMotorPos) != 1 && londonInput > 0) {
                    londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                    londonMotorOutput = (londonInput * Global_Variables.londonLiftGain);            //Apply motor velocity to match trigger inputs

                    //londonMotor.setTargetPosition(londonMotor.getCurrentPosition());            //Set the motor target to wherever it is now
                } else if (CheckLondonLimit(londonMotorPos) != -1 && londonInput < 0) {
                    londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);                   //Set motor to run at velocity
                    londonMotorOutput = (londonInput* Global_Variables.londonLowerGain);            //Apply motor velocity to match trigger inputs
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





    void ManualLiftControl(){
        if (liftPower != 0) {                                                        //Check if value is not zero
            if(liftMotor.getCurrentPosition() < Global_Variables.maxViperUnderMinBeforeLift && liftPower > 0 && londonMotor.getCurrentPosition() < Global_Variables.minBeforeExt) {
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            } else if (liftMotor.getCurrentPosition() < Global_Variables.viperMax && liftPower > 0 && londonMotor.getCurrentPosition() > Global_Variables.minBeforeExt) {
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            } else if(viperMinMagSwitch.getState() && liftPower < 0){
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(liftPower);                                          //Apply motor power to match trigger inputs
                liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now

            } else if (!viperMinMagSwitch.getState() && liftPower < 0) {
                liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            } else{
                if(londonMotor.getCurrentPosition() > Global_Variables.minBeforeExt) {
                    if (liftMotor.getCurrentPosition() > Global_Variables.viperMax) {
                        liftMotor.setTargetPosition(Global_Variables.viperMax);
                    }
                }
                if(londonMotor.getCurrentPosition() < Global_Variables.minBeforeExt) {
                    if (liftMotor.getCurrentPosition() > Global_Variables.maxViperUnderMinBeforeLift) {
                        liftMotor.setTargetPosition(Global_Variables.maxViperUnderMinBeforeLift);
                    }
                }

                if(liftMotor.getCurrentPosition() < Global_Variables.viperMin){
                    liftMotor.setTargetPosition(Global_Variables.viperMin);
                }
                liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                   //Set motor to run to target position
                liftMotor.setPower(1);
                // liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);               //Set motor to run with power setting
                //liftMotor.setPower(0);                                          //Apply motor power to match trigger inputs
                //liftMotor.setTargetPosition(liftMotor.getCurrentPosition());            //Set the motor target to wherever it is now
            }
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
        } else {                                                                     //If above value is 0 (no gamepad input)
            if(londonMotor.getCurrentPosition() > Global_Variables.minBeforeExt) {
                if (liftMotor.getCurrentPosition() > Global_Variables.viperMax) {
                    liftMotor.setTargetPosition(Global_Variables.viperMax);
                }
            }
            if(londonMotor.getCurrentPosition() < Global_Variables.minBeforeExt) {
                if (liftMotor.getCurrentPosition() > Global_Variables.maxViperUnderMinBeforeLift) {
                    liftMotor.setTargetPosition(Global_Variables.maxViperUnderMinBeforeLift);
                }
            }

            if(liftMotor.getCurrentPosition() < Global_Variables.viperMin){
                liftMotor.setTargetPosition(Global_Variables.viperMin);
            }
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);                   //Set motor to run to target position
            liftMotor.setPower(1);                                                  //Allow full power
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
