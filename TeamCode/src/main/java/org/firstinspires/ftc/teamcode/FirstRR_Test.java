package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

@Autonomous(name = "First RR Test")//Set Program Name and Mode
public final class FirstRR_Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-36, -60, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);



        waitForStart();
        while (1 == 1 && !isStopRequested()) {
            Action TrajectoryAction2 = drive.actionBuilder(drive.pose)
                    //.lineTo(new Vector2d(0, 0), Math.toRadians(90))
                    //.splineToConstantHeading(new Vector2d(0, 0), Math.toRadians(90))
                    .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(90))
                    .build();


            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction2
                    )
            );
            Action TrajectoryAction3 = drive.actionBuilder(drive.pose)
                    //.splineTo(new Vector2d(-36, -60), Math.toRadians(90))
                    //.splineToConstantHeading(new Vector2d(-36, -60), Math.toRadians(90))
                    //.strafeTo(new Vector2d(-36, -60))
                    .strafeToLinearHeading(new Vector2d(-36, -60), Math.toRadians(90))
                    .build();
            Actions.runBlocking(
                    new SequentialAction(
                            TrajectoryAction3
                    )
            );

        }
    }
    }

