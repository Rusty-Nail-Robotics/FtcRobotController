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
    public Servo ledServo;
    public Servo gripServo;
    public DcMotorEx londonMotor;
    public int liftTarget = 0;
    public int liftVelocity = 300;
    public int londonTarget = 0;

    void Setup(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        londonMotor = hardwareMap.get(DcMotorEx.class, "londonMotor");
        londonMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        londonMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        //gripServo = hardwareMap.get(Servo.class, "GripServo");
        //gripServo.setPosition(0);

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setPower(1);
        liftMotor.setTargetPosition(liftTarget);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        londonMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        londonMotor.setPower(1);
        londonMotor.setTargetPosition(londonTarget);
        londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);



    }

   void Operations(HardwareMap hardwareMap, LinearOpMode linearOpMode){
        double liftPower = linearOpMode.gamepad1.left_trigger + -linearOpMode.gamepad1.right_trigger;
       if(liftPower != 0) {
           liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
           liftMotor.setPower(liftPower);
           liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
       }else {

           liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
           liftMotor.setPower(1);
       }

    if(linearOpMode.gamepad1.left_stick_y != 0) {
        londonMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        londonMotor.setVelocity(linearOpMode.gamepad1.left_stick_y*150);
        londonMotor.setTargetPosition(londonMotor.getCurrentPosition());
    }else {

        londonMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }
}
}
