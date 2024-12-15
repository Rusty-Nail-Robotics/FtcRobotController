package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driver_Control")//Set Program Name and Mode
public class Drive extends LinearOpMode {
    @Override

    public void runOpMode() {
        //Initialization begins here

        //Create container to hold the Non_Move program and set it up
        Non_Move non_move = new Non_Move();                         //Create the container named non_move
        non_move.Setup(hardwareMap,this);               //run the setup function in Non_Move

        //Create container to hold the Show_Off program
        Show_Off showOff = new Show_Off();                          //Create the container named showoff
        showOff.Setup(hardwareMap, this);               //run the setup function in Show_Off

        //Create container to hold Chassis_Control program
        Chassis_Control chassisControl = new Chassis_Control();     //Create the container named chassiscontrol
        chassisControl.Setup(hardwareMap, this);        //Run the Setup function in Chassis_Control


        waitForStart();                                             //Wait for the Play button to be pressed on the driver station
        while (opModeIsActive()) {                                  //Loop for as long as the op mode is active
            non_move.LiftOperations(hardwareMap, this);     //Run the Operations function in Non_Move
            showOff.SetLedPattern(hardwareMap, this);   //Run the SetLedPattern function in Show_Off
            chassisControl.TelopDrive(hardwareMap, this);   //Run the TelopDrive function in Chassis_Control
            non_move.TelopGripperOperations(hardwareMap, this);     // Run the TelopGripperOperations function in Non_Move
            telemetry.update();

        }
    }


}