package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.util.ElapsedTime;



@Autonomous(name = "Basket Side")//Set Program Name and Mode
public final class Basket_Side_Auto extends LinearOpMode {
    Non_Move non_move = new Non_Move();
    ElapsedTime gameTime = new ElapsedTime();
    ElapsedTime gpTimer = new ElapsedTime();


    @Override

    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -60, Math.toRadians(90));




        non_move.Setup(hardwareMap, this);               //run the setup function in Non_Move

        Show_Off showOff = new Show_Off();                          //Create the container named showoff
        showOff.Setup(hardwareMap, this);               //run the setup function in Show_Off

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        Action StartToSub = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.sub_Spec_Set_Location[0], Autonomous_Variables.sub_Spec_Set_Location[1]), Math.toRadians(Autonomous_Variables.sub_Spec_Set_Location[2]))
                .build();
        non_move.londonMotor.setPower(.5);

        waitForStart();
        gameTime.reset();

            //// Place on High Chamber//
        Global_Variables.basketMode = 3;
        non_move.AutoLiftOperations(this);
        Actions.runBlocking(
                new SequentialAction(
                        StartToSub
                )
        );

        non_move.londonMotor.setTargetPosition(Autonomous_Variables.subDeliverLondonTarget - Autonomous_Variables.subDeliverLondonDropDistance);
        WaitSeconds(.5);
        non_move.liftMotor.setTargetPosition(Autonomous_Variables.subDeliverLiftTarget - Autonomous_Variables.subDeliverRetractionDistance);

        WaitSeconds(.75);

        non_move.gripServo.setPosition(Global_Variables.gripperOpen);

        WaitSeconds(.15);

        //Pick up Inner Sample//

        Global_Variables.basketMode = 2;
        non_move.AutoLiftOperations(this);

        Action PickInnerFromFloor = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Inner_Location[0], Autonomous_Variables.pick_From_Ground_Inner_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Inner_Location[2]))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickInnerFromFloor
                )
        );

        non_move.liftMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLiftTarget);
        while (non_move.liftMotor.getCurrentPosition() < 900) {
            telemetry.update();//do nothing
        }
        non_move.londonMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLondonTarget);
        WaitSeconds(1);
        non_move.gripServo.setPosition(Global_Variables.gripperClosed);

        //Place Inner Sample in High Basket

        WaitSeconds(.5);
        non_move.londonMotor.setPower(1);
        non_move.londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);

        Action PrepPlaceInBasket = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .turnTo(Math.toRadians(225))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PrepPlaceInBasket
                )
        );
        Global_Variables.basketMode = 1;
        while (non_move.liftMotor.getCurrentPosition() < Global_Variables.maxViperUnderMinBeforeLift + 100) {
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

//Pick Middle Sample from floor

        Action PickCenterFromFloor = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .afterDisp(0, RetractLift())
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Center_Location[0], Autonomous_Variables.pick_From_Ground_Center_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Center_Location[2]))

                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickCenterFromFloor
                )
        );
        non_move.liftMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLiftTarget);

        WaitSeconds(2);
        non_move.londonMotor.setPower(.35);
        non_move.londonMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLondonTarget);
        WaitSeconds(1.5);
        non_move.gripServo.setPosition(Global_Variables.gripperClosed);
        WaitSeconds(.5);

        //Place middle sample in High Basket

        non_move.londonMotor.setPower(1);
        non_move.londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);

        Action PrepPlaceInBasketCent = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .turnTo(Math.toRadians(225))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PrepPlaceInBasketCent
                )
        );
        Global_Variables.basketMode = 1;
        while (non_move.liftMotor.getCurrentPosition() < Global_Variables.maxViperUnderMinBeforeLift + 100) {
            non_move.AutoLiftOperations(this);
        }
        Action PlaceInBasketCent = drive.actionBuilder(drive.pose)
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.place_In_Basket_Location[0], Autonomous_Variables.place_In_Basket_Location[1]), Math.toRadians(Autonomous_Variables.place_In_Basket_Location[2]))
                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PlaceInBasketCent
                )
        );
        non_move.gripServo.setPosition(Global_Variables.gripperOpen);


        // Pick Outer Sample from floor

        Action PickOuterFromFloor = drive.actionBuilder(drive.pose)
                .strafeTo(new Vector2d(Autonomous_Variables.safe_Pivot_Point[0],Autonomous_Variables.safe_Pivot_Point[1]))
                .afterDisp(0, RetractLift())
                .strafeToLinearHeading(new Vector2d(Autonomous_Variables.pick_From_Ground_Outer_Location[0], Autonomous_Variables.pick_From_Ground_Outer_Location[1]), Math.toRadians(Autonomous_Variables.pick_From_Ground_Outer_Location[2]))

                .build();
        Actions.runBlocking(
                new SequentialAction(
                        PickOuterFromFloor
                )
        );
        non_move.liftMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLiftTarget);
        WaitSeconds(2);
        non_move.londonMotor.setPower(.35);
        non_move.londonMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLondonTarget);
        WaitSeconds(1.5);
        non_move.gripServo.setPosition(Global_Variables.gripperClosed);
        WaitSeconds(.5);
        non_move.londonMotor.setPower(1);
        non_move.londonMotor.setTargetPosition(Global_Variables.basketLondonTarget);







        gpTimer.reset();
        while (gpTimer.seconds() < 10 && !isStopRequested()) {
            telemetry.addData("Waiting ", gpTimer.seconds());
            telemetry.update();
        }
        non_move.londonMotor.setTargetPosition(50);
        gpTimer.reset();
        while (gpTimer.seconds() < 2 && !isStopRequested()) {
            telemetry.addData("Waiting ", gpTimer.seconds());
            telemetry.update();
        }
        non_move.ParkLondon();

    }


    public void WaitSeconds(double seconds) {
        gpTimer.reset();
        while (gpTimer.seconds() < seconds && !isStopRequested()) {
            telemetry.update();
            //Do Nothing
        }
    }

    Action RetractLift(){
        non_move.liftMotor.setTargetPosition(Autonomous_Variables.pickFromGroundLiftTarget);
        return RetractLift();
    }


}



