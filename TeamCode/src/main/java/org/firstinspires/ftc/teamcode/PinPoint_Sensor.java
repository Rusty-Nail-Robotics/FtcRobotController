package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

public class PinPoint_Sensor {
    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer


    public void PinPoint_Setup (HardwareMap hardwareMap, LinearOpMode linearOpMode){
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.resetPosAndIMU();

        linearOpMode.telemetry.addData("Status", "Initialized");
        linearOpMode.telemetry.addData("X offset", odo.getXOffset());
        linearOpMode.telemetry.addData("Y offset", odo.getYOffset());
        linearOpMode.telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        linearOpMode.telemetry.addData("Device Scalar", odo.getYawScalar());
        linearOpMode.telemetry.update();

    }

    public double[] Read_PinPoint_Position(){
        odo.update();
        Pose2D pos = odo.getPosition();
        //String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        double[] PositionArray_X_Y_RX = {pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES)};
        //telemetry.addData("Position", data);
        return PositionArray_X_Y_RX;
    }
}
