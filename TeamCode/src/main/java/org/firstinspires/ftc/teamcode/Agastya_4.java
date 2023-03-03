package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.servoclass;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@TeleOp
public class Agastya_4 extends LinearOpMode {
    //pole alligning parameters
    Rev2mDistanceSensor backLeftDist,backRightDist;
    public static double PoleAlign_ASSUMED_MID = 3;
    public static double PoleAlign_DISTANCE_THRESHOLD = 1.5;
    public static double PoleAlign_factor = 1;

    RevColorSensorV3 colorSensorGripper;
    boolean alignPole = false;
    boolean colorSensorUsed = false;
    boolean colorSensorUsedButton = true;

    Pose2d startPose = new Pose2d(24+12+4, -(24+24+12+4), Math.toRadians(180));
    DcMotorEx liftleft,liftright;
    double throttle = 0.6;
    double strafe = 0.6;
    double turn = 0.4;
    double pos = 0;
    double uppower = 1;

    //below distance sensor parameters
    Rev2mDistanceSensor distanceSensorLeft,distanceSensorRight;
    public static double FrontAlign_target = 4.75 + 1;//2.5;
    public static double FrontAlign_DISTANCE_THRESHOLD = 1;//0.75;
    public static double FrontAlign_factor = 1;

    double sliderDrop = Constants.sliderDrop;
    double sliderPickUp = Constants.sliderPickUp;
    int low = Constants.lowHeight;
    int middle = Constants.middleHeight;
    int high = Constants.highHeight;//1650;

    Servo podsHolder;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensorGripper = hardwareMap.get(RevColorSensorV3.class,"colorSensorGripper");
        colorSensorGripper.enableLed(true);

        podsHolder = hardwareMap.get(Servo.class,"podsHolder");

        distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorLeft");
        distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorRight");

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeftDist = hardwareMap.get(Rev2mDistanceSensor.class,"backLeftDist");
        backRightDist = hardwareMap.get(Rev2mDistanceSensor.class,"backRightDist");

        liftleft = hardwareMap.get(DcMotorEx.class, "liftleft");
        liftright = hardwareMap.get(DcMotorEx.class, "liftright");

        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setDirection(DcMotorSimple.Direction.REVERSE);

        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);

        gripping.setservo(gripping.servorotate, Constants.rotatorPickUP);
        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
        gripping.setservo(gripping.servogripper, Constants.gripperOpen);

        TrajectorySequence up= drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                }).waitSeconds(0.1)
                .addTemporalMarker(()->{
                    moveLifter(pos);
                    gripping.setservo(gripping.servoslide, sliderDrop);
                }).waitSeconds(0.75)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .build();

        TrajectorySequence down= drive.trajectorySequenceBuilder(new Pose2d())
                .waitSeconds(0.01)
                .UNSTABLE_addTemporalMarkerOffset(0,() ->gripping.setservo(gripping.servogripper, Constants.gripperClosed))
                .UNSTABLE_addTemporalMarkerOffset(0.7,() ->gripping.setservo(gripping.servoslide, sliderPickUp))
                .waitSeconds(0.71)
                .UNSTABLE_addTemporalMarkerOffset(-0.01,() ->gripping.setservo(gripping.servogripper, Constants.gripperOpen))
                .build();

        TrajectorySequence lowdown= drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                }).waitSeconds(0.2)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servoslide, sliderPickUp);
                }).waitSeconds(0.3)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                }).waitSeconds(0.3)
                .addTemporalMarker(()->{
                    moveLifter(pos);
                }).waitSeconds(0.3)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                })
//                .addTemporalMarker(()->{
//                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
//                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
//                }).waitSeconds(0.75)
//                .addTemporalMarker(()->{
//                    moveLifter(pos);
//                }).waitSeconds(0.1)
//                .addTemporalMarker(()->{
//                    gripping.setservo(gripping.servoslide, sliderPickUp);
//                }).waitSeconds(0.5)
//                .addTemporalMarker(()->{
//                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
//                })
                .build();


        TrajectorySequence lowdownFromLow= drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    moveLifter(middle);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                }).waitSeconds(0.75)
                .addTemporalMarker(()->{
                    moveLifter(pos);
                }).waitSeconds(0.1)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servoslide, sliderPickUp);
                }).waitSeconds(0.5)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                })
                .build();

        TrajectorySequence coneDrop= drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    moveLifter(pos);
                }).waitSeconds(1)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                })
                .build();

        TrajectorySequence conePickup= drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                }).waitSeconds(0.3)
                .addTemporalMarker(()->{
                    moveLifter(pos);
                })
                .build();

        drive.setPoseEstimate(startPose);

        podsHolder.setPosition(Constants.podsUp);

        waitForStart();


        while (opModeIsActive()) {
            // drop angle position setting : player 2
            if(gamepad2.left_bumper)
            {
                sliderDrop = Constants.sliderDropIncline;
                low = Constants.lowHeightIncline;
                middle = Constants.midddleHeightIncline;
                high = Constants.highHeightIncline;
            }
            else if(gamepad2.right_bumper)
            {
                sliderDrop = Constants.sliderDrop;
                low = Constants.lowHeight;
                middle = Constants.middleHeight;
                high = Constants.highHeight;
            }

            // speed changing on holding dpad up button : player 2
            if(gamepad1.right_stick_button)
            {
                throttle = 1;
                strafe = 1;
                turn = 1;
            }
            else
            {
                throttle = 0.6;
                strafe = 0.6;
                turn = 0.4;
            }

            if (gamepad1.right_bumper) {
                pos=high;
                drive.followTrajectorySequenceAsync(up);

            } else if (gamepad1.right_trigger > 0.8) {
                pos = middle;
                drive.followTrajectorySequenceAsync(up);

            } else if (gamepad1.left_trigger > 0.8) {
                pos=low;
                drive.followTrajectorySequenceAsync(up);

            }
            //moving lifter to ground from any position other than low
            else if (gamepad1.left_bumper) {
                pos=0;
                drive.followTrajectorySequenceAsync(lowdown);
            }
            //moving lifter to ground from low position
            else if(gamepad1.back)
            {
                pos = 0;
                drive.followTrajectorySequenceAsync(lowdownFromLow);
            }

//            if (gamepad1.back) {
//                pos=150;
//                drive.followTrajectorySequenceAsync(down);
//            }

            if(gamepad1.y){
                pos += 30;
                moveLifter(pos);
            }
            if(gamepad1.a){
                pos -= 30;
                moveLifter(pos);
            }

            //gripper rotator angle
            if(gamepad1.dpad_up)
            {
                gripping.setservo(gripping.servoslide,gripping.servoslide.getPosition() -  0.05);
            }
            else if(gamepad1.dpad_down)
            {
                gripping.setservo(gripping.servoslide,gripping.servoslide.getPosition() +  0.05);
            }

            //encoder reset in low position
            if(gamepad1.start)
            {
                telemetry.addLine("encoder reset");
                liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                pos = 0;
            }

            //Pole aligning
            if(gamepad2.dpad_left) alignPole = true;
            else if(gamepad2.dpad_right) alignPole = false;
            if(alignPole == true){
                distanceAlign(drive,FrontAlign_target);
                poleAlign(drive);
            }

            //Gripper test
            if (gamepad1.b) {//CLAW Open
                if(pos >= 150)
                {
                    pos -= 50;
                    drive.followTrajectorySequenceAsync(coneDrop);
                }
                else
                {
                    gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                }
            }

            //cone stack pickup
            if(gamepad2.y)
            {
                pos = Constants.cone1;
                moveLifter(pos);
            }
            else if(gamepad2.b)
            {
                pos = Constants.cone2;
                moveLifter(pos);
            }
            else if(gamepad2.a)
            {
                pos = Constants.cone3;
                moveLifter(pos);
            }
            else if(gamepad2.x)
            {
                pos = Constants.cone4;
                moveLifter(pos);
            }

            if (gamepad1.x) {
                if(pos < 100)
                {
                    pos += 200;
                    drive.followTrajectorySequenceAsync(conePickup);
                }
                else
                {
                    gripping.setservo(gripping.servogripper, Constants.gripperClosed);
                }
            }

            if(gamepad2.dpad_up) colorSensorUsedButton = true;
            else if(gamepad2.dpad_down) colorSensorUsedButton = false;


            if(pos <= 0 && colorSensorUsedButton == true)
            {
                colorSensorUsed = true;
            }
            else
            {
                colorSensorUsed = false;
            }

            if(colorSensorUsed==true && (colorSensorGripper.blue() > 240 || colorSensorGripper.red()>165))
            {
                pos += 200;

                drive.followTrajectorySequenceAsync(conePickup);
//                gripping.setservo(gripping.servogripper, Constants.gripperClosed);
            }


            telemetry.addData("pos = ",pos);
            telemetry.addData("servo slide",gripping.servoslide.getPosition());
            telemetry.addData("throttle",throttle);
            telemetry.addData("strafe",strafe);
            telemetry.addData("liftleft", liftleft.getCurrentPosition());
            telemetry.addData("liftright", liftright.getCurrentPosition());
            telemetry.addData("lift L", liftleft.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("lift R", liftright.getCurrent(CurrentUnit.AMPS));

            //pose telemetry
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            //drive power
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y * throttle,
                            -gamepad1.left_stick_x * strafe,
                            -gamepad1.right_stick_x * turn
                    ));
            drive.update();
        }
    }

    void moveLifter(double pos)
    {
        liftleft.setTargetPosition((int) pos);
        liftleft.setPower(uppower);
        liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftright.setTargetPosition((int) pos);
        liftright.setPower(uppower);
        liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    void poleAlign(SampleMecanumDrive drive)
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


    void distanceAlign(SampleMecanumDrive drive,double FrontAlign_target1)
    {
        while (opModeIsActive())
        {
            double leftSensorDist = distanceSensorLeft.getDistance(DistanceUnit.INCH);
            double rightSensorDist = distanceSensorRight.getDistance(DistanceUnit.INCH);
            double error = Math.min(leftSensorDist,rightSensorDist) - FrontAlign_target1;

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
