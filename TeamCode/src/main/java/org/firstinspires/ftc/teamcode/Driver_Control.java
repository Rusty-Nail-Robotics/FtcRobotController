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
        waitForStart();
        while (opModeIsActive()) {
           chassisControl.DriveSystem(hardwareMap, this);
            double[] currentPosition = odometery.Read_PinPoint_Position();
            telemetry.addData("X location = ", currentPosition[0]);
            telemetry.addData("Y Location = ", currentPosition[1]);
            telemetry.addData("Heading = ", currentPosition[2]);
            telemetry.addData("random", Math.random());
            telemetry.update();
            telemetryOutput.Drive_Telem(hardwareMap, this, chassisControl);

        }
    }


}