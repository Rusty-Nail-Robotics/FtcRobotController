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



@Autonomous(name = "Basket Side")//Set Program Name and Mode

public final class Non_Basket_Side_Auto extends LinearOpMode {

    Non_Move non_move = new Non_Move();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime gpTimer = new ElapsedTime();
    RevBlinkinLedDriver blinkinLedDriver;           //create container holding the led driver info


    @Override

    public void runOpMode() {// Removed this to clear an error " throws InterruptedException { "
        Pose2d beginPose = new Pose2d(24, -60, Math.toRadians(90));
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

                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.getSub_Spec_Set_Location_Right[0], Autonomous_Variables.getSub_Spec_Set_Location_Right[1]), Math.toRadians(Autonomous_Variables.getSub_Spec_Set_Location_Right[2]))

                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance, 1))
                //.waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                //.waitSeconds(.15)
                .stopAndAdd(new MotorTarget(non_move.liftMotor,Global_Variables.viperMin,1))
                .stopAndAdd(new MotorTarget(non_move.londonMotor,0,1))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_1[0],Autonomous_Variables.jump_Point_1[1]),Math.toRadians(Autonomous_Variables.jump_Point_1[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_2[0],Autonomous_Variables.jump_Point_2[1]),Math.toRadians(Autonomous_Variables.jump_Point_2[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.push_Point_1[0],Autonomous_Variables.push_Point_1[1]),Math.toRadians(Autonomous_Variables.push_Point_1[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_2[0],Autonomous_Variables.jump_Point_2[1]),Math.toRadians(Autonomous_Variables.jump_Point_2[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_3[0],Autonomous_Variables.jump_Point_3[1]),Math.toRadians(Autonomous_Variables.jump_Point_3[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.push_Point_2[0],Autonomous_Variables.push_Point_2[1]),Math.toRadians(Autonomous_Variables.push_Point_2[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_3[0],Autonomous_Variables.jump_Point_3[1]),Math.toRadians(Autonomous_Variables.jump_Point_3[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.jump_Point_4[0],Autonomous_Variables.jump_Point_4[1]),Math.toRadians(Autonomous_Variables.jump_Point_4[2]))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.push_Point_3[0],Autonomous_Variables.push_Point_3[1]),Math.toRadians(Autonomous_Variables.push_Point_3[2]))

                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.clip_Grab_Point[0],Autonomous_Variables.clip_Grab_Point[1]),Math.toRadians(Autonomous_Variables.clip_Grab_Point[2]))
                .waitSeconds(1.3)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .waitSeconds(.7)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget,.37))
                .waitSeconds(.15)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.15)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget, 1))


                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.getSub_Spec_Set_Location_Right[0], Autonomous_Variables.getSub_Spec_Set_Location_Right[1]), Math.toRadians(Autonomous_Variables.getSub_Spec_Set_Location_Right[2]))

                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance, 1))
                //.waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                //.waitSeconds(.15)


                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.clip_Grab_Point[0],Autonomous_Variables.clip_Grab_Point[1]),Math.toRadians(Autonomous_Variables.clip_Grab_Point[2]))
                .waitSeconds(1.3)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .waitSeconds(.7)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget,.37))
                .waitSeconds(.15)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.15)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget, 1))


                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.getSub_Spec_Set_Location_Right[0], Autonomous_Variables.getSub_Spec_Set_Location_Right[1]), Math.toRadians(Autonomous_Variables.getSub_Spec_Set_Location_Right[2]))

                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance, 1))
                //.waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                //.waitSeconds(.15)


                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.clip_Grab_Point[0],Autonomous_Variables.clip_Grab_Point[1]),Math.toRadians(Autonomous_Variables.clip_Grab_Point[2]))
                .waitSeconds(1.3)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.pickFromGroundLiftTarget, 1))
                .waitSeconds(.7)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.pickFromGroundLondonTarget,.37))
                .waitSeconds(.15)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperClosed))
                .waitSeconds(.15)
                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget, 1))
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget, 1))


                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.getSub_Spec_Set_Location_Right[0], Autonomous_Variables.getSub_Spec_Set_Location_Right[1]), Math.toRadians(Autonomous_Variables.getSub_Spec_Set_Location_Right[2]))

                .stopAndAdd(new MotorTarget(non_move.londonMotor, Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance, 1))
                //.waitSeconds(.25)
                .stopAndAdd(new MotorTarget(non_move.liftMotor, Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance, 1))
                .waitSeconds(.25)
                .stopAndAdd(new ServoTarget(non_move.gripServo, Global_Variables.gripperOpen))
                //.waitSeconds(.15)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.push_Point_3[0],Autonomous_Variables.push_Point_3[1]),Math.toRadians(Autonomous_Variables.push_Point_3[2]))


                .build();
        Actions.runBlocking(
                new SequentialAction(
                        StartToSub
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
    public static class CheckLiftLimit implements Action{ //made static to clear error
        DcMotorEx lift;
        DcMotorEx london;
        RevBlinkinLedDriver ledDriver;


        public CheckLiftLimit(RevBlinkinLedDriver ledDriver, DcMotorEx lift, DcMotorEx london){
            this.london = london;
            this.lift = lift;
            this.ledDriver = ledDriver;

        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {

            if(london.getCurrentPosition() < Global_Variables.minBeforeExt && lift.getCurrentPosition() > Global_Variables.maxViperUnderMinBeforeLift){
                new LedControl(ledDriver,1);
            }

            return false;
        }
    }
}



