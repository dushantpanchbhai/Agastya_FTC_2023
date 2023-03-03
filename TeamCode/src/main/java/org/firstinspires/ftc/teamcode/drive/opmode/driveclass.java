package org.firstinspires.ftc.teamcode.drive.opmode;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.RUN_USING_ENCODER;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

public class driveclass {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    public  driveclass(HardwareMap hardwareMap){
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
//        lift = hardwareMap.get(DcMotorEx.class, "lift");

//


        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        // TODO: reverse any motors using DcMotor.setDirection()
//        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
//
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void drive(double x,double y,double rx,double maxspeed){
//        rx=-rx;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx),1);
        double frontLeftPower = ((y + x + rx) / denominator)*maxspeed;
        double backLeftPower = ((y - x + rx) / denominator)*maxspeed;
        double frontRightPower = ((y - x - rx) / denominator)*maxspeed;
        double backRightPower = ((y + x - rx) / denominator)*maxspeed;

        leftFront.setPower(frontLeftPower);
      leftRear.setPower(backLeftPower);
       rightFront.setPower(frontRightPower);
        rightRear.setPower(backRightPower);
    }
}
