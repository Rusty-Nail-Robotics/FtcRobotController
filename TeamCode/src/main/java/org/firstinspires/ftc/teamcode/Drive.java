package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Driver_Control")//Set Program Name and Mode
public class Drive extends LinearOpMode {
    @Override

    public void runOpMode() {
        Non_Move non_move = new Non_Move();
        non_move.Setup(hardwareMap,this);
        Show_Off showOff = new Show_Off();
        showOff.Setup(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            non_move.Operations(hardwareMap, this);
            showOff.SetLedPattern(hardwareMap, this);


        }
    }


}