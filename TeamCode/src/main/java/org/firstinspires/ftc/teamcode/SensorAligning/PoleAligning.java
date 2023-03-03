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
public class PoleAligning extends LinearOpMode {
    Rev2mDistanceSensor backLeftDist;
    Rev2mDistanceSensor backRightDist;

    public static double ASSUMED_MID = 3;
    public static double DISTANCE_THRESHOLD = 2;
    public static double factor = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeftDist = hardwareMap.get(Rev2mDistanceSensor.class,"backLeftDist");
        backRightDist = hardwareMap.get(Rev2mDistanceSensor.class,"backRightDist");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);

        gripping.setservo(gripping.servorotate, Constants.rotatorPickUP);
        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
        gripping.setservo(gripping.servogripper, Constants.gripperOpen);

        Pose2d startPose = new Pose2d(41, -63, Math.toRadians(180));
        drive.setPoseEstimate(startPose);

        waitForStart();

        while (opModeIsActive())
        {
            double distanceLeft = backLeftDist.getDistance(DistanceUnit.INCH);
            double distanceRight = backRightDist.getDistance(DistanceUnit.INCH);

            if(distanceLeft >= DISTANCE_THRESHOLD && distanceRight >= DISTANCE_THRESHOLD)
            {
                telemetry.addLine("bot stable");
            }
            else
            {
                double x,y,heading;
                //if leaned towards right
                if(distanceLeft >= DISTANCE_THRESHOLD && distanceRight <= DISTANCE_THRESHOLD)
                {
                    double error = ASSUMED_MID - distanceRight;
                    x = drive.getPoseEstimate().getX() + (error*factor)*(Math.sin(drive.getPoseEstimate().getHeading()));
                    y = drive.getPoseEstimate().getY() + (error*factor)*(-1*Math.cos(drive.getPoseEstimate().getHeading()));
                    heading = drive.getPoseEstimate().getHeading();
                }
                //if leaned towards left
                else
                {
                    double error = ASSUMED_MID - distanceLeft;
//                    x = drive.getPoseEstimate().getX();
//                    y = drive.getPoseEstimate().getY() - (error / 2.0);
                    x = drive.getPoseEstimate().getX() - (error*factor)*(Math.sin(drive.getPoseEstimate().getHeading()));
                    y = drive.getPoseEstimate().getY() - (error*factor)*(-1*Math.cos(drive.getPoseEstimate().getHeading()));
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

//    public double wallSensorPID(double current, double target){
//        double error = current - target;
//        double pError = error;
//        double dError = error - prevError_wall;
//        double Ierror = error + prevError_wall;
//
//        prevError_wall = error;
//        return pError * kp_wall + dError * kd_wall + Ierror * ki_wall;
//    }
}
