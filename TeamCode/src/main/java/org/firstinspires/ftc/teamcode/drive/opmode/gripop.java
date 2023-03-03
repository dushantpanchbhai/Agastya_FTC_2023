package org.firstinspires.ftc.teamcode.drive.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(group = "drive")


public class gripop extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        gripper servogrip=new gripper();
        servogrip.init(hardwareMap);
        waitForStart();

        while (!isStopRequested()) {
            if (gamepad1.x){
              servogrip.setservo(servogrip.grip, 0.2);
            }
            if (gamepad1.a){
                servogrip.setservo(servogrip.grip, 0.8);
            }
            if (gamepad1.y){
                servogrip.setservo(servogrip.grip, 0);
            }
        }
    }
}
