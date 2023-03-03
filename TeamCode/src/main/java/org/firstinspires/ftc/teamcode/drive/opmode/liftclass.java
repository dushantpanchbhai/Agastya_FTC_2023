package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class liftclass {
    DcMotorEx liftleft,liftright;
    double maxspeed,downspeed,uppower;

    int lowestpole,middlepole,highestpole;
    public void init(HardwareMap hardwareMap) {
        DcMotorEx liftleft = hardwareMap.get(DcMotorEx.class, "liftleft");
        DcMotorEx liftright = hardwareMap.get(DcMotorEx.class, "liftright");

        double maxspeed = 0.4,downspeed=1,uppower=1;

        int lowestpole=680,middlepole=1280,highestpole=1700;


        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        liftleft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    public void up(String pos){
        int target=0;
        if(pos=="low"){
            target=lowestpole;
        }
        else if(pos=="mid"){
            target=middlepole;
        }
        else if(pos=="top"){
            target=highestpole;
        }

        while(liftleft.isBusy()) {

            liftleft.setTargetPosition(target);
            liftleft.setTargetPositionTolerance(15);
            liftleft.setPower(uppower);
            liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftright.setTargetPosition(target);
            liftright.setPower(uppower);
            liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }


    }
    public void down(){
        while(liftleft.getCurrentPosition()>20) {
            liftleft.setTargetPosition(0);
            liftleft.setPower(downspeed);
            liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftright.setTargetPosition(0);
            liftright.setPower(downspeed);
            liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}
