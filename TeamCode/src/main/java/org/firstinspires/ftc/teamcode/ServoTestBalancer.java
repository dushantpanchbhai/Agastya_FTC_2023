package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.servoclass;

@TeleOp
public class ServoTestBalancer extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);
        gripping.servoBalancer.setPosition(0.5);
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                 gripping.servoBalancer.setPosition(gripping.servoBalancer.getPosition() + 0.01);
            }
            else if(gamepad1.dpad_down)
            {
                gripping.servoBalancer.setPosition(gripping.servoBalancer.getPosition() - 0.01);
            }

            sleep(50);

            telemetry.addData("servo balancer pos",gripping.servoBalancer.getPosition());
            telemetry.update();
        }
    }
}
