package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name = "Red Basket Side")//Set Program Name and Mode

public final class Basket_Side_Red_Auto extends LinearOpMode {

    Non_Move non_move = new Non_Move();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime gpTimer = new ElapsedTime();
    RevBlinkinLedDriver blinkinLedDriver;           //create container holding the led driver info


    @Override

    public void runOpMode() {// Removed this to clear an error " throws InterruptedException { "
        Pose2d beginPose = new Pose2d(Autonomous_Variables.basket_Side_Red[0], Autonomous_Variables.basket_Side_Red[1], Math.toRadians(Autonomous_Variables.basket_Side_Red[2]));
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "ledServo");  //Associate in-program name with robot configuration name

        non_move.Setup(hardwareMap, this);               //run the setup function in Non_Move
        Show_Off showOff = new Show_Off();                          //Create the container named showoff
        showOff.Setup(hardwareMap, this);               //run the setup function in Show_Off
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



         waitForStart();
        gameTime.reset();

            //// Place on High Chamber//
        Action StartToSub = drive.actionBuilder(drive.pose)
                .stopAndAdd(new LedControl(blinkinLedDriver,0))
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget, .5))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget, 1))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.sub_Spec_Set_Location[0], Autonomous_Variables.sub_Spec_Set_Location[1]), Math.toRadians(Autonomous_Variables.sub_Spec_Set_Location[2]))
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance, 1))
                //.waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                //.waitSeconds(.15)

                .build();

        Actions.runBlocking(
                new SequentialAction(
                        StartToSub
                )
        );


        //Pick up Inner Sample and place in high basket//



        Action PickInnerFromFloor = drive.actionBuilder(drive.pose)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.fastDownLiftTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Global_Variables.fastDownLondonTarget, .25))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Inner_Location[0], Autonomous_Variables.pick_From_Ground_Inner_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Inner_Location[2]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .waitSeconds(.5)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Global_Variables.basketLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.maxViperUnderMinBeforeLift, 1))
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.basketLiftTarget, 1))
                .turnTo(Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.place_In_Basket_Location[0], Autonomous_Variables.place_In_Basket_Location[1]), Math.toRadians(Autonomous_Variables.place_In_Basket_Location[2]))
                .waitSeconds(.35)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickInnerFromFloor
                )
        );

//Pick Middle Sample from floor and place in high basket

        Action PickCenterFromFloor = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Center_Location[0], Autonomous_Variables.pick_From_Ground_Center_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Center_Location[2]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .waitSeconds(.5)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget, .35))
                .waitSeconds(1.5)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Global_Variables.basketLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.maxViperUnderMinBeforeLift, 1))
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.basketLiftTarget, 1))
                .turnTo(Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.place_In_Basket_Location[0], Autonomous_Variables.place_In_Basket_Location[1]), Math.toRadians(Autonomous_Variables.place_In_Basket_Location[2]))
                .waitSeconds(.35)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickCenterFromFloor
                )
        );



        // Pick Outer Sample from floor

        Action PickOuterFromFloor = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Outer_Location[0], Autonomous_Variables.pick_From_Ground_Outer_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Outer_Location[2]))
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget + 50, .35))
                .waitSeconds(1.5)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget + Autonomous_Variables.getPickOuterExtraReach, 1))
                .waitSeconds(.75)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget, .35))
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Global_Variables.basketLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.maxViperUnderMinBeforeLift, 1))
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Global_Variables.basketLiftTarget, 1))
                .turnTo(Math.toRadians(225))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.place_In_Basket_Location[0], Autonomous_Variables.place_In_Basket_Location[1]), Math.toRadians(Autonomous_Variables.place_In_Basket_Location[2]))
                .waitSeconds(.35)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickOuterFromFloor
                )
        );














        gpTimer.reset();
        while (gpTimer.seconds() < 2 && !isStopRequested()) {
            telemetry.addData("Waiting ", gpTimer.seconds());
            telemetry.update();
        }
        non_move.ParkLondon();

    }






  public static class MotorTarget implements Action{  //made static to clear error
        DcMotorEx motor;
        int target;
        double power;

        public MotorTarget(DcMotorEx m, int t, double p){
            this.motor = m;
            this.target = t;
            this.power = p;
        }
      @Override
      public boolean run(@NonNull TelemetryPacket telemetryPacket) {
          motor.setPower(power);
          motor.setTargetPosition(target);
          return false;
      }
  }
    public static class ServoTarget implements Action{ //made static to clear error
        Servo servo;
        double target;


        public ServoTarget(Servo m, double t){
            this.servo = m;
            this.target = t;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            servo.setPosition(target);
            return false;
        }
    }

    public static class LedControl implements Action { //made static to clear error
        RevBlinkinLedDriver ledDriver;
        int mode;


        public LedControl(RevBlinkinLedDriver ledDriver, int mode) {
            this.ledDriver = ledDriver;
            this.mode = mode;

        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            switch (Global_Variables.ledMode) {
                case 0:
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
                case 1:
                    ledDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_RED);
            }

            return false;
        }

    }

}



