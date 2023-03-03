package org.firstinspires.ftc.teamcode.SensorAligning;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Disabled
@TeleOp
public class ColorSensorPositioning extends LinearOpMode {
    RevColorSensorV3 colorSensorLeft,colorSensorRight;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class,"colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class,"colorSensorRight");

        colorSensorRight.enableLed(true);
        colorSensorLeft.enableLed(true);

        waitForStart();

        while (opModeIsActive())
        {
            telemetry.addData("color sensor left blue",colorSensorLeft.blue());
            telemetry.addData("color sensor right blue",colorSensorRight.blue());
            telemetry.addData("color sensor left",colorSensorLeft.red() +" "+ colorSensorLeft.green() +" "+ colorSensorLeft.blue());
            telemetry.addData("color sensor right",colorSensorLeft.red() +" "+ colorSensorLeft.green() +" "+ colorSensorLeft.blue());
            telemetry.addData("detected left color",colorSensorLeft.toString());
            telemetry.addData("detected right color",colorSensorRight.toString());
            telemetry.update();
        }
    }
}
