package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class gripper {

Servo grip;
double home_position;
    public void init(HardwareMap hardwareMap) {
    home_position=0.5;
    grip=hardwareMap.servo.get("servogripper");

        grip.setPosition(home_position);

    }
    public void setservo(Servo servo,double pos){
        servo.setPosition(pos);
    }




}
