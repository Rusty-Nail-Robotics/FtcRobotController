package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "Driver_Control")//Set Program Name and Mode
public class Driver_Control extends LinearOpMode {
    @Override

    public void runOpMode() {

        Telop_Chassis_Control chassisControl = new Telop_Chassis_Control();
        chassisControl.TopOperationsSetup(hardwareMap);
        PinPoint_Sensor odometery = new PinPoint_Sensor();
        odometery.PinPoint_Setup(hardwareMap, this);
        Telemetry_Output telemetryOutput = new Telemetry_Output();
        //waitForStart();
        while (!opModeIsActive() && !isStopRequested()){
            double[] currentPosition = odometery.Read_PinPoint_Position();
            telemetryOutput.Add_Odometry_Output(hardwareMap, this, currentPosition);
            telemetry.update();
        }
        while (opModeIsActive()) {
            telemetry.addData("random", Math.random());
           chassisControl.DriveSystem(hardwareMap, this);
            double[] currentPosition = odometery.Read_PinPoint_Position();
            telemetryOutput.Add_Odometry_Output(hardwareMap,this, currentPosition);
            telemetryOutput.Add_Drive_Telem(hardwareMap, this, chassisControl);
            telemetry.update();

        }
    }


}