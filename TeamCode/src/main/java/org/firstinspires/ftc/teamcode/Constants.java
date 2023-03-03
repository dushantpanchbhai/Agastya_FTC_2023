package org.firstinspires.ftc.teamcode;
import com.acmerobotics.dashboard.config.Config;

@Config
public class Constants {
    public static double sliderDrop = 0.819;//0.779;//0.58;
    public static double sliderPickUp = 0.189;//0.139;//0.2688;//0.2088;
    public static double sliderDropIncline = 0.758;//0.748;

    public static double rotatorPickUP = 0.81;//0.82;//1;//0.86;//1;
    public static double rotatorDrop = 0.1;//0.1394;//0;//0.1094;//0;

    public static int lowHeight = 680;
    public static int middleHeight = 1180;
    public static int highHeight = 1700;
    public static int lowHeightIncline = 440;
    public static int midddleHeightIncline = 1010;
    public static int highHeightIncline = 1480;

    public static double gripperClosed = 0.35;
    public static double gripperOpen = 0.57;//0.55;

    public static double balancerZero = 1;//0.219;
    public static double balancerOut  = 0.32;//0.25;//0.149;
    public static double balancerIn   = 0.12;//0.319;

    public static double podsDown = 0.478;
    public static double podsUp = 0.217;

    public static int cone1 = 300;
    public static int cone2 = 240;
    public static int cone3 = 140;
    public static int cone4 = 100;
    public static int cone5 = 0;

//    5 : 300;
//    4 : 240;
//    3 : 140;
//    2 : 100;
//    1 : 0

    // [][-] servo : start and back: (g2.left_bumper and g2.right_bumper)
    // [testing done][-] dpad buttons for cone stack left : 5 se clockwise ([][dpad up and down used in adjusting servo slide])
    // [testing done][-] y, b, a : high middle low
    // [testing done][-] right_bumpter : high, right trigger : medium , left_trigger : low;
    // [testing done][-] game2 : x , left_trigger for speed changer
    // [testing done][-] gamepad1 - dpad_left,right : rotator ,,, if gamepad2 left_bumper is pressed then - rotator position reset.

    // reduce servo angle increment
}
