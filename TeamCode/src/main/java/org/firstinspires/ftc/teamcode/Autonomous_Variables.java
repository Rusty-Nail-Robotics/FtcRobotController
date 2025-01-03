package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class Autonomous_Variables {

    ///////////////// Start Points ////////////////
    public static double[] basket_Side_Red ={-36,-60,90};
    public static double[] non_Basket_Side_Red = {24,-60,90};
    //public static double[] basket_Side_Blue = {-36,-60,90};
    //public static double[] non_Basket_Side_Blue = {24,-60,90};

/////////////////// All Auto Programs  ///////////////////

    public static int subDeliverLiftTarget = 1650;
    public static int subDeliverLondonTarget = 700;
    public static int subDeliverLondonDropDistance = 120;
    public static int subDeliverRetractionDistance = 900;


    /////////////////// Basket Side //////////////////////
    public static double[] sub_Spec_Set_Location = {-10,-36,90};
    public static double[] pick_From_Ground_Inner_Location = {-47, -41, 90};
    public static double[] pick_From_Ground_Center_Location ={-55,-41,90};
    public static double[] pick_From_Ground_Outer_Location = {-48,-24,180};
    public static int pickFromGroundLondonTarget = -100;
    public static int pickFromGroundLiftTarget = 1650;
    public static int getPickOuterExtraReach = 200;

    public static double[] safe_Pivot_Point = {-46,-46};
    public static double[] place_In_Basket_Location = {-48,-48,225};

    /////////////////// Non Basket Side //////////////////////
    public static double[] getSub_Spec_Set_Location_Right = {10,-36,90};
    public static double[] jump_Point_1 = {36,-36,180};
    public static double[] jump_Point_2 = {48,-7,180};
    public static double[] push_Point_1 = {48,-52,180};
    public static double[] jump_Point_3 = {54,-7,180};
    public static double[] push_Point_2 = {54,-52,180};
    public static double[] jump_Point_4 = {58,-7,-180};
    public static double[] push_Point_3 = {56,-52,180};
    public static double[] clip_Grab_Point = {32,-44,320};   // used as parking location
    public static double[] jump_Point_1__2 = {36,-10,180};

}
