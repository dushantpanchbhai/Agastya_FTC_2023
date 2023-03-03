package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class servoclass {
    public Servo servogripper;
    public Servo servorotate;
    public Servo servoslide;
    public Servo servoBalancer;
    public final static double home=0.5;
    public void init(HardwareMap hardwareMap) {

        servogripper=hardwareMap.servo.get("servogripper");
        servorotate=hardwareMap.servo.get("servorotate");
        servoslide=hardwareMap.servo.get("servoslide");
        servoBalancer = hardwareMap.servo.get("balancer");
    }

    public void setservo(Servo servo,double pos){
        servo.setPosition(pos);
    }

    public void grip(){
        servogripper.setPosition(0.3);
    }
    public void release(){
        servogripper.setPosition(0.5);
    }

}
