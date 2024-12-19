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
        if(linearOpMode.gamepad1.dpad_up){
            Global_Variables.basketMode = 1;
        }


        switch (Global_Variables.basketMode){
            case 0:
                linearOpMode.telemetry.addData("Manual Mode", 1);
                break;

            case 1:
                if(londonMotor.getCurrentPosition() < Global_Variables.minBeforeExt){
                    londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);
                    londonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    londonMotor.setPower(1);
                }else{
                    londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);
                    londonMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    londonMotor.setPower(1);
                    liftMotor.setTargetPosition(Global_Variables.basketLiftTarget);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                }
                if(linearOpMode.gamepad1.left_stick_y != 0 || linearOpMode.gamepad1.dpad_down || linearOpMode.gamepad1.left_trigger != 0 || linearOpMode.gamepad1.right_trigger != 0){
                    Global_Variables.basketMode = 0;
            }

        }

        if (londonMotor.getCurrentPosition()<Global_Variables.minBeforeExt && liftMotor.getCurrentPosition() > Global_Variables.maxViperUnderMinBeforeLift){
            liftMotor.setTargetPosition(Global_Variables.maxViperUnderMinBeforeLift);
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
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
