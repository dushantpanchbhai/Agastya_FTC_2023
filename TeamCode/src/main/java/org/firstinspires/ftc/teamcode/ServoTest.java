package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.servoclass;

@TeleOp
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);
        gripping.servoslide.setPosition(0.5);
        gripping.servorotate.setPosition(0);
        waitForStart();

        while (opModeIsActive())
        {
            if(gamepad1.dpad_up)
            {
                 gripping.servoslide.setPosition(gripping.servoslide.getPosition() - 0.01);
            }
            else if(gamepad1.dpad_down)
            {
                gripping.servoslide.setPosition(gripping.servoslide.getPosition() + 0.01);
            }
            else if(gamepad1.dpad_left)
            {
                gripping.servorotate.setPosition(gripping.servorotate.getPosition() + 0.01);
            }
            else if(gamepad1.dpad_right)
            {
                gripping.servorotate.setPosition(gripping.servorotate.getPosition() - 0.01);
            }

            sleep(50);

            telemetry.addData("servo slider pos",gripping.servoslide.getPosition());
            telemetry.addData("servo rotate pos",gripping.servorotate.getPosition());
            telemetry.update();
        }
    }
}
