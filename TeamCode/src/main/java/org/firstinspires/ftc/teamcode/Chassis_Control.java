package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Chassis_Control {
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;


    public double overCurrentOverride = 1;
    public double y = 0;
    public double x = 0;
    public double rx = 0;
    public double frontLeftPower = 0;
    public double backLeftPower = 0;
    public double frontRightPower = 0;
    public double backRightPower = 0;


    public void Setup(HardwareMap hardwareMap, LinearOpMode ignoredLinearOpMode) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftRear.setDirection(DcMotor.Direction.REVERSE);
        //rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void TelopDrive(HardwareMap ignoredHardwareMap, LinearOpMode linearOpMode) {
        if (leftFront.isOverCurrent() || rightFront.isOverCurrent() || leftRear.isOverCurrent() || rightRear.isOverCurrent()) {
            Global_Variables.overCurrentOverride = .4;
            Global_Variables.ledMode = 1;
        } else {
            overCurrentOverride = 1;
        }

        if(linearOpMode.gamepad1.right_bumper){
            Global_Variables.speedOverride = Global_Variables.speedOverrideSlow;
        }//else {
           // Global_Variables.speedOverride = Global_Variables.highSpeed;
        //}



        y = (((-linearOpMode.gamepad1.right_stick_x) * Global_Variables.speedOverride) * overCurrentOverride); // Remember, this is reversed!
        x = (((linearOpMode.gamepad1.right_stick_y) * Global_Variables.speedOverride) * overCurrentOverride); // Counteract imperfect strafing
        rx = ((((-linearOpMode.gamepad1.left_stick_x) * Global_Variables.speedOverride) * 1) * overCurrentOverride);


        linearOpMode.telemetry.addData("y = ", y);
        linearOpMode.telemetry.addData("x = ", x);
        linearOpMode.telemetry.addData("rx = ", rx);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
         frontLeftPower = (y + x - rx) / denominator;
         backLeftPower = (y - x + rx) / denominator;
         frontRightPower = (y - x - rx) / denominator;
         backRightPower = (y + x + rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);



    }
}
