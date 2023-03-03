package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp
public class ColorSensorGripping extends LinearOpMode {
    RevColorSensorV3 colorSensorGripper;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorGripper = hardwareMap.get(RevColorSensorV3.class,"colorSensorGripper");
        colorSensorGripper.enableLed(true);

        waitForStart();

        //blue : 240;
        //red : 165

        while (opModeIsActive())
        {
            telemetry.addData("blue", colorSensorGripper.blue());
            telemetry.addData("red",colorSensorGripper.red());
            telemetry.addData("green",colorSensorGripper.green());
            telemetry.update();
        }
    }
}
