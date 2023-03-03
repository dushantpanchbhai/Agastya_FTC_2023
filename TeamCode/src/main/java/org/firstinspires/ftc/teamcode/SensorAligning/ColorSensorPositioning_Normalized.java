package org.firstinspires.ftc.teamcode.SensorAligning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Disabled
@Config
@TeleOp
public class ColorSensorPositioning_Normalized extends LinearOpMode {
    RevColorSensorV3 colorSensorLeft, colorSensorRight;

    public static int threshold = 175;
    public static double power = 0.21;


    public static double kp_gyro = 0.03;//0.04;
    public static double kd_gyro = 0;//0.01;
    public static double ki_gyro = 0;
    public static double prevError_gyro = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");

        colorSensorRight.enableLed(true);
        colorSensorLeft.enableLed(true);

        waitForStart();

        while (opModeIsActive()) {
            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = Math.toDegrees(angleWrap(poseEstimate.getHeading()));
            double headingPower = gyroPID(heading, 0);

            if (Math.abs(headingPower) < 0.09) {
                headingPower = 0;
            }

            if (colorSensorLeft.blue() > threshold && colorSensorRight.blue() > threshold) {
                drive.setWeightedDrivePower(new Pose2d(0, 0, -headingPower));
                telemetry.addLine("at position");
            } else if (colorSensorLeft.blue() > threshold && colorSensorRight.blue() < threshold) {
                drive.setWeightedDrivePower(new Pose2d(0, -power,0));
            } else if (colorSensorLeft.blue() < threshold && colorSensorRight.blue() > threshold) {
                drive.setWeightedDrivePower(new Pose2d(0, power, 0));
            }

            drive.update();

            telemetry.addData("color sensor left blue", colorSensorLeft.blue());
            telemetry.addData("color sensor right blue", colorSensorRight.blue());
            telemetry.addData("color sensor left", colorSensorLeft.red() + " " + colorSensorLeft.green() + " " + colorSensorLeft.blue());
            telemetry.addData("color sensor right", colorSensorLeft.red() + " " + colorSensorLeft.green() + " " + colorSensorLeft.blue());
            telemetry.update();
        }
    }

    public double gyroPID(double current, double target)
    {
        double error = current - target;
        double pError = error;
        double dError = error - prevError_gyro;
        double Ierror = error + prevError_gyro;

        prevError_gyro = error;

        return pError * kp_gyro + dError * kd_gyro + Ierror * ki_gyro;
    }

    public double angleWrap(double radians)
    {
        while (radians > Math.PI)
        {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI)
        {
            radians += 2 * Math.PI;
        }
        return radians;
    }
}