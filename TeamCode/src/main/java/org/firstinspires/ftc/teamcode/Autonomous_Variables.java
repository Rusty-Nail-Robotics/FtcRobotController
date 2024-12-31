package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Autonomous_Variables {


    /////////////////// Basket Side //////////////////////
    public static double[] sub_Spec_Set_Location = {-6,-36,90};
    public static int subDeliverLiftTarget = 1650;
    public static int subDeliverLondonTarget = 700;
    public static int subDeliverLondonDropDistance = 120;
    public static int subDeliverRetractionDistance = 900;


    public static double[] pick_From_Ground_Inner_Location = {-46, -41, 90};
    public static double[] pick_From_Ground_Center_Location ={-55,-41,90};
    public static double[] pick_From_Ground_Outer_Location = {-48,-22,180};
    public static int pickFromGroundLondonTarget = -100;
    public static int pickFromGroundLiftTarget = 1650;

    public static double[] safe_Pivot_Point = {-46,-46};
    public static double[] place_In_Basket_Location = {-48,-48,225};
}
