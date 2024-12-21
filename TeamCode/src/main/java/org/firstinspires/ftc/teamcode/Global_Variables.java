package org.firstinspires.ftc.teamcode;

public class Global_Variables {
     //General values
     static double speedOverride = .25;
     static double speedOverrideSlow = .75;
     static double highSpeed = 1;
     static double overCurrentOverride = 0.4;
     static double gripperOpen = 1;
     static double gripperClosed = 0;
     static int ledMode = 0;
     static double londonLiftGain = 750;
     static double londonLowerGain = 450;
     static int basketMode = 0;

     //Min/Max locations for lift/extention
     static int londonMinAtExtention = -248;

     static int londonMaxAtExtention = 1300;

     static int viperMax = 3000;
     static int viperMin = 0;
     static int minBeforeExt = 350; //london motor location where 3rd stage of viper slid can come out
     static int basketLondonTarget = 800;
     static int basketLiftTarget = 3000;
     static int maxViperUnderMinBeforeLift = 1000;
     static int fastDownLiftTarget = maxViperUnderMinBeforeLift-50;  //where to retract arm to when down arrow is pressed
     static int fastDownLondonTarget = 0;



     //Autonumous values

}
