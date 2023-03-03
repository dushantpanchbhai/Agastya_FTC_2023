package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class PodsTest extends LinearOpMode {
    Servo pods;

    @Override
    public void runOpMode() throws InterruptedException {
        pods = hardwareMap.get(Servo.class,"podsHolder");
        pods.setPosition(0.5);
        waitForStart();
        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                pods.setPosition(pods.getPosition() + 0.01);
            }
            else if(gamepad1.dpad_down)
            {
                pods.setPosition(pods.getPosition() - 0.01);
            }

            sleep(20);


            telemetry.addData("pods position",pods.getPosition());
            telemetry.update();
        }
    }
}
