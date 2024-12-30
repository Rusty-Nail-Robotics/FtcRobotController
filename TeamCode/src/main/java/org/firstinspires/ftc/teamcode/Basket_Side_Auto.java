package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "Basket Side")//Set Program Name and Mode
public final class Basket_Side_Auto extends LinearOpMode {

    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -60, Math.toRadians(90));

        ElapsedTime gameTime = new ElapsedTime();
        ElapsedTime gpTimer = new ElapsedTime();

        Non_Move non_move = new Non_Move();
        non_move.Setup(hardwareMap,this);               //run the setup function in Non_Move

        Show_Off showOff = new Show_Off();                          //Create the container named showoff
        showOff.Setup(hardwareMap, this);               //run the setup function in Show_Off

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Action StartToSub = drive.actionBuilder(drive.pose)
                //.lineTo(new Vector2d(0, 0), Math.toRadians(90))
                //.splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.sub_Spec_Set_Location[0], Autonomous_Variables.sub_Spec_Set_Location[1]), Math.toRadians(Autonomous_Variables.sub_Spec_Set_Location[2]))
                .build();
            non_move.londonMotor.setPower(.5);

        waitForStart();
        gameTime.reset();


            Global_Variables.basketMode = 3;
            non_move.AutoLiftOperations(this);
            Actions.runBlocking(
                    new SequentialAction(
                            StartToSub
                    )
            );
        gpTimer.reset();
        while(gpTimer.seconds() < .5  && !isStopRequested()){
            //Do Nothing
        }
        non_move.londonMotor.setTargetPosition(Autonomous_Variables.subDeliverLondonTarget- Autonomous_Variables.subDeliverLondonDropDistance);
        non_move.liftMotor.setTargetPosition(Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance);

        gpTimer.reset();
        while(gpTimer.seconds() < .5  && !isStopRequested()){
                //Do Nothing
            }

        non_move.gripServo.setPosition(Global_Variables.gripperOpen);

        gpTimer.reset();
        while(gpTimer.seconds() < .5 && !isStopRequested()){
                //Do Nothing
            }

            Global_Variables.basketMode = 2;
            non_move.AutoLiftOperations(this);

            /// Pick From Floor
        Action PickInnerFromFloor = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Inner_Location[0], Autonomous_Variables.pick_From_Ground_Inner_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Inner_Location[2]))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickInnerFromFloor
                )
        );

        non_move.liftMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLiftTarget);
        while(non_move.liftMotor.getCurrentPosition()<900){DoNothing();}
        non_move.londonMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLondonTarget);
        gpTimer.reset();
        while(gpTimer.seconds() < .5 && !isStopRequested()){
            //Do Nothing
        }
        non_move.gripServo.setPosition(Global_Variables.gripperClosed);


        gpTimer.reset();
        while(gpTimer.seconds() < .5 && !isStopRequested()){
            //Do Nothing
        }

        non_move.londonMotor.setTargetPosition(30);

        Action PrepPlaceInBasket = drive.actionBuilder(drive.pose)
                .turnTo(Math.toRadians(225))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PrepPlaceInBasket
                )
        );
        Global_Variables.basketMode=1;
        while(non_move.liftMotor.getCurrentPosition() < Global_Variables.maxViperUnderMinBeforeLift + 100){
            non_move.AutoLiftOperations(this);
        }
        Action PlaceInBasket = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.place_In_Basket_Location[0], Autonomous_Variables.place_In_Basket_Location[1]), Math.toRadians(Autonomous_Variables.place_In_Basket_Location[2]))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PlaceInBasket
                )
        );
        non_move.gripServo.setPosition(Global_Variables.gripperOpen);




        gpTimer.reset();
        while(gpTimer.seconds() < 2 && !isStopRequested()){
            //Do Nothing
        }
















        gpTimer.reset();
            while(gpTimer.seconds() < 10 && !isStopRequested()){
                telemetry.addData("Waiting ", gpTimer.seconds());
                telemetry.update();
            }
            non_move.londonMotor.setTargetPosition(50);
            gpTimer.reset();
        while(gpTimer.seconds() < 2 && !isStopRequested()){
            telemetry.addData("Waiting ", gpTimer.seconds());
            telemetry.update();
        }
            non_move.ParkLondon();

            }

            public void DoNothing(){

            }
}

