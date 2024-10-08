package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;


public class Telop_Chassis_Control {
    public DcMotorEx leftFront;
    public DcMotorEx leftRear;
    public DcMotorEx rightRear;
    public DcMotorEx rightFront;
    public double overCurrentOverride = 1;
    public double y = 0;
    public double x = 0;
    public double rx = 0;
    public double frontLeftPower = 0;
    public  double backLeftPower = 0;
    public  double frontRightPower = 0;
    public  double backRightPower = 0;


    public void TopOperationsSetup(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void DriveSystem(HardwareMap ignoredhardwareMap, LinearOpMode linearOpMode){
        double speedOverride = 1;

        if(leftFront.isOverCurrent() || rightFront.isOverCurrent() || leftRear.isOverCurrent() || rightRear.isOverCurrent()){
            overCurrentOverride = .4;
            linearOpMode.telemetry.addData("!!!Drive Motors Overloaded!!!!!!",0);
        }else{
            overCurrentOverride = 1;
        }


        y = (((-linearOpMode.gamepad1.right_stick_y) * speedOverride) * overCurrentOverride); // Remember, this is reversed!
        x = (((linearOpMode.gamepad1.right_stick_x) * speedOverride) * overCurrentOverride); // Counteract imperfect strafing
        rx = ((((linearOpMode.gamepad1.left_stick_x) * speedOverride) * 1)* overCurrentOverride);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        leftFront.setPower(frontLeftPower);
        leftRear.setPower(backLeftPower);
        rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);


    }






















}