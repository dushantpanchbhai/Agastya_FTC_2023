package org.firstinspires.ftc.teamcode.SensorAligning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.servoclass;

@Config
@TeleOp
public class DistanceAligningBack extends LinearOpMode {
    Rev2mDistanceSensor distanceSensorBack;

    public static double BackAlign_target = 1;
    public static double BackAlign_DISTANCE_THRESHOLD = 0.5;
    public static double BackAlign_factor = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensorBack = hardwareMap.get(Rev2mDistanceSensor.class,"distanceSensor");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);

        Pose2d startPose = new Pose2d(41, -63, Math.toRadians(180));

        gripping.setservo(gripping.servorotate, Constants.rotatorPickUP);
        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
        gripping.setservo(gripping.servogripper, Constants.gripperOpen);

        drive.setPoseEstimate(startPose);

        waitForStart();

        distanceAlignBack(drive);

        telemetry.addLine("bot is stable");
        telemetry.update();
    }

    void distanceAlignBack(SampleMecanumDrive drive) {
        while (opModeIsActive()) {
            double backDistance = distanceSensorBack.getDistance(DistanceUnit.INCH);
            double error = backDistance - BackAlign_target;

            if (error >= 10) return;

            if (Math.abs(error) > BackAlign_DISTANCE_THRESHOLD) {
                // Calculate the desired position and orientation
                double x = drive.getPoseEstimate().getX() + (error * BackAlign_factor);
                double y = drive.getPoseEstimate().getY();
                double heading = drive.getPoseEstimate().getHeading();

                // Move the bot to the desired position and orientation using Road Runner
                if (drive.isBusy() == false) {
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineTo(new Vector2d(x, y))
                            .build());
                }
            } else {
                telemetry.addLine("bot stable");
                return;
            }

            drive.update();
            telemetry.addData("distance at back: ", distanceSensorBack.getDistance(DistanceUnit.INCH));
            telemetry.addData("error", error);
            telemetry.update();
        }
    }
}
