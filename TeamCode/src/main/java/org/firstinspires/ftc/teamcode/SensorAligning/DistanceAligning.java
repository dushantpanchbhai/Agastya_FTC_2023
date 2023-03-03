package org.firstinspires.ftc.teamcode.SensorAligning;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.path.heading.ConstantInterpolator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Cos;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive_Pods;
import org.firstinspires.ftc.teamcode.drive.servoclass;

@Config
@TeleOp
public class DistanceAligning extends LinearOpMode {
    Rev2mDistanceSensor distanceSensorLeft,distanceSensorRight;
    Rev2mDistanceSensor backLeftDist,backRightDist;

    public static double FrontAlign_target = 2.5;
    public static double FrontAlign_DISTANCE_THRESHOLD = 0.75;
    public static double FrontAlign_factor = 1;

    public static double PoleAlign_ASSUMED_MID = 3;
    public static double PoleAlign_DISTANCE_THRESHOLD = 2;
    public static double PoleAlign_factor = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorLeft");
        distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorRight");

        backLeftDist = hardwareMap.get(Rev2mDistanceSensor.class,"backLeftDist");
        backRightDist = hardwareMap.get(Rev2mDistanceSensor.class,"backRightDist");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);

        Pose2d startPose = new Pose2d(41, -63, Math.toRadians(180));

        gripping.setservo(gripping.servorotate, Constants.rotatorPickUP);
        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
        gripping.setservo(gripping.servogripper, Constants.gripperOpen);

        drive.setPoseEstimate(startPose);

        waitForStart();

        PoleAlign(drive);
        DistanceAlign(drive);

        telemetry.addLine("bot is stable");
        telemetry.update();
    }

    void PoleAlign(SampleMecanumDrive drive)
    {
        while (opModeIsActive())
        {
            double distanceLeft = backLeftDist.getDistance(DistanceUnit.INCH);
            double distanceRight = backRightDist.getDistance(DistanceUnit.INCH);

            if(distanceLeft >= PoleAlign_DISTANCE_THRESHOLD && distanceRight >= PoleAlign_DISTANCE_THRESHOLD)
            {
                telemetry.addLine("bot stable");
                return;
            }
            else
            {
                double x,y,heading;
                //if leaned towards right
                if(distanceLeft >= PoleAlign_DISTANCE_THRESHOLD && distanceRight <= PoleAlign_DISTANCE_THRESHOLD)
                {
                    double error = PoleAlign_ASSUMED_MID - distanceRight;
                    x = drive.getPoseEstimate().getX() + (error*PoleAlign_factor)*(Math.sin(drive.getPoseEstimate().getHeading()));
                    y = drive.getPoseEstimate().getY() + (error*PoleAlign_factor)*(-1*Math.cos(drive.getPoseEstimate().getHeading()));
                    heading = drive.getPoseEstimate().getHeading();
                }
                //if leaned towards left
                else
                {
                    double error = PoleAlign_ASSUMED_MID - distanceLeft;
//                    x = drive.getPoseEstimate().getX();
//                    y = drive.getPoseEstimate().getY() - (error / 2.0);
                    x = drive.getPoseEstimate().getX() - (error*PoleAlign_factor)*(Math.sin(drive.getPoseEstimate().getHeading()));
                    y = drive.getPoseEstimate().getY() - (error*PoleAlign_factor)*(-1*Math.cos(drive.getPoseEstimate().getHeading()));
                    heading = drive.getPoseEstimate().getHeading();
                }

                if(drive.isBusy() == false)
                {
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineTo(new Vector2d(x, y))
                            .build());
                }
            }


            drive.update();
            telemetry.addData("left Sensor Dist",backLeftDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("right Sensor Dist",backRightDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }

    void DistanceAlign(SampleMecanumDrive drive)
    {
        while (opModeIsActive())
        {
            double leftSensorDist = distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double rightSensorDist = distanceSensorRight.getDistance(DistanceUnit.INCH);
            double error = Math.min(leftSensorDist,rightSensorDist) - FrontAlign_target;

            if(error >= 10) return;

            if (Math.abs(error) > FrontAlign_DISTANCE_THRESHOLD) {
                // Calculate the desired position and orientation
                double x = drive.getPoseEstimate().getX() + (error*FrontAlign_factor)*(Math.cos(drive.getPoseEstimate().getHeading()));
                double y = drive.getPoseEstimate().getY() + (error*FrontAlign_factor)*(1*Math.sin(drive.getPoseEstimate().getHeading()));
                double heading = drive.getPoseEstimate().getHeading();

                // Move the bot to the desired position and orientation using Road Runner
                if(drive.isBusy() == false)
                {
                    drive.followTrajectory(drive.trajectoryBuilder(drive.getPoseEstimate())
                            .lineTo(new Vector2d(x, y))
                            .build());
                }
            }
            else
            {
                telemetry.addLine("bot stable");
                return;
            }

            drive.update();
            telemetry.addData("distance front left: ",distanceSensorLeft.getDistance(DistanceUnit.INCH));
            telemetry.addData("distance front right: ",distanceSensorRight.getDistance(DistanceUnit.INCH));
            telemetry.addData("distance left",backLeftDist.getDistance(DistanceUnit.INCH));
            telemetry.addData("distance right",backRightDist.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}
