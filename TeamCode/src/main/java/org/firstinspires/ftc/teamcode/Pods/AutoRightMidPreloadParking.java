package org.firstinspires.ftc.teamcode.Pods;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.drive.servoclass;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.Arrays;

@Config
@Autonomous
public class AutoRightMidPreloadParking extends LinearOpMode {
    DcMotorEx liftleft,liftright;
    double uppower = 0.9;

    //Timer
    ElapsedTime timer = new ElapsedTime();
    double elapsedSeconds = timer.seconds();

    //camera setup...
    OpenCvCamera camera;
    org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline AprilTagDetectionPipeline;
    int PARKING_ZONE1 = 0, PARKING_ZONE2 = 1, PARKING_ZONE3 = 2;
    int[] eureka_IDS = {3, 7, 9};
    AprilTagDetection tagOfInterest = null;
    // for lens calibiration
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    int ID_TAG_OF_INTEREST = 18;
    static final double FEET_PER_METER = 3.28084;
    String ParkingZone = "None";
    WebcamName webcamName;
    //

    //below color sensor parameters
    RevColorSensorV3 colorSensorLeft, colorSensorRight;
    public static int threshold = 175;
    public static double power = 0.21;
    public static double kp_gyro = 0.04;
    public static double kd_gyro = 0.01;
    public static double ki_gyro = 0;
    public static double prevError_gyro = 0;

    //below distance sensor parameters
    Rev2mDistanceSensor distanceSensorLeft,distanceSensorRight;
    public static double FrontAlign_target = 4.75 + 1.75 - 0.25 -2;//5.5;//4.5 + 1 ;//2.5; // this is when cone is their
    public static double FrontAlign_target_Pole = FrontAlign_target + 1;
    public static double FrontAlign_DISTANCE_THRESHOLD = 1;//0.75;
    public static double FrontAlign_factor = 1;

    // below distance sensor parameters for wall
    Rev2mDistanceSensor distanceSensorBack;
    public static double BackAlign_target = 1;
    public static double BackAlign_DISTANCE_THRESHOLD = 0.8;//0.5;
    public static double BackAlign_factor = 1;

    //pole alligning parameters
    Rev2mDistanceSensor backLeftDist,backRightDist;
    public static double PoleAlign_ASSUMED_MID = 3;
    public static double PoleAlign_DISTANCE_THRESHOLD = 2;
    public static double PoleAlign_factor = 1.2;

    //
    int low = 680;
    int middle = 1180;
    public static int high = 1550;

    int cone1 = Constants.cone1;
    int cone2 = Constants.cone2;
    int cone3 = Constants.cone3;
    int cone4 = Constants.cone4;
    int cone5 = Constants.cone5;

    double gripperClosed = 0.25;
    double gripperOpen = 0.55;

    int pickUpPos = 285;
    public static double sliderDrop = 0.742;//Constants.sliderDrop;
    double sliderPickUp = Constants.sliderPickUp;

    Pose2d startPose = new Pose2d(41-1, -63, Math.toRadians(180));


    Pose2d dropPose1 = new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135));
    Pose2d dropPose2 = new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135));
    Pose2d dropPose3;// = new Pose2d(24+3+1 -0.25,-(24-3+1) +0.25,Math.toRadians(-135));
    Pose2d dropPose4;// = new Pose2d(24+3+1 -0.75,-(24-3+1) +0.75,Math.toRadians(-135));

    double pickUpCoordinateX1 = 24+24+12+3.5+0.5;
    double pickUpCoordinateY1 = -11.5;

    double pickUpCorrdinateY2 = -11.5 + 0.5;
    double pickUpCorrdinateY3 = -11.5 + 1;
    double pickUpCorrdinateY4 = -11.5 + 1.5;

    Vector2d pickUpCoordinateChanged;

    double DropDelay = 0.75;

    double bufferWaitTime = 1;
    double endTime = 25;

    @Override
    public void runOpMode() throws InterruptedException {

        //camera setup/////////////////////
        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);
        AprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(AprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("error in camera");
            }
        });
        /////////////////////
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        liftleft = hardwareMap.get(DcMotorEx.class, "liftleft");
        liftright = hardwareMap.get(DcMotorEx.class, "liftright");
        liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //reversing lift left
        liftleft.setDirection(DcMotorSimple.Direction.REVERSE);
        //
        liftright.setDirection(DcMotorSimple.Direction.REVERSE);

        servoclass gripping = new servoclass();
        gripping.init(hardwareMap);

        colorSensorLeft = hardwareMap.get(RevColorSensorV3.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(RevColorSensorV3.class, "colorSensorRight");
        colorSensorRight.enableLed(true);
        colorSensorLeft.enableLed(true);

        distanceSensorLeft = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorLeft");
        distanceSensorRight = hardwareMap.get(Rev2mDistanceSensor.class,"FrontDistanceSensorRight");

        backLeftDist = hardwareMap.get(Rev2mDistanceSensor.class,"backLeftDist");
        backRightDist = hardwareMap.get(Rev2mDistanceSensor.class,"backRightDist");

        distanceSensorBack = hardwareMap.get(Rev2mDistanceSensor.class,"distanceSensor");

        drive.setPoseEstimate(startPose);

        TrajectoryVelocityConstraint fastConstraint = new MinVelocityConstraint(Arrays.asList(
                new TranslationalVelocityConstraint(45),
                new AngularVelocityConstraint(35)
        ));

        TrajectorySequence PreloadDrop = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,gripperClosed);
                }).waitSeconds(0.1)
                .addTemporalMarker(()->{
                    lifterUp(middle);
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                }).waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                })
//                // Moving to Pole
//                .splineToConstantHeading(new Vector2d(24+11,-(24+24)),Math.toRadians(90))
//                .strafeTo(new Vector2d(24+11,-(24+12)))
//                .splineToConstantHeading(new Vector2d(24+5+.2 +1, -24-.2 - 1), Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(24+11,-(24+24)),Math.toRadians(90))
                .strafeTo(new Vector2d(24+11,-24))
                .splineToSplineHeading(new Pose2d(24+8,-24+8,Math.toRadians(-135)),Math.toRadians(+120))
//                .lineTo(new Vector2d(24+5,-24+5))
                .build();


        TrajectorySequence PreloadDrop2 = drive.trajectorySequenceBuilder(PreloadDrop.end())
                //OpeningGripper
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                }).waitSeconds(0.75)
////                ##############################################################
                //
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerZero);
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                    lifterUp(cone1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                    gripping.setservo(gripping.servoslide, sliderPickUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{
                    gripping.setservo(gripping.servogripper,gripperOpen);
                })
                //
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(24+12+5,-11.5),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24+12+2,-11.5),Math.toRadians(90))
                .lineTo(new Vector2d(24+24+12+3.5+0.5,-11.5))
                .build();

        TrajectorySequence Cone1PickUp = drive.trajectorySequenceBuilder(PreloadDrop2.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,gripperClosed);
                }).waitSeconds(0.4)
                .addTemporalMarker(()->{
                    lifterUp(cone1+400);
                }).waitSeconds(0.1)
                //
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                    lifterUp(middle);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                })
                //---- motion -------
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
                .build();

        TrajectorySequence Cone1Drop = drive.trajectorySequenceBuilder(Cone1PickUp.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerIn);
                }).waitSeconds(.5)
                .addTemporalMarker(()->{
//                    lifterUp(middle - 200);
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                }).waitSeconds(DropDelay)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerZero);
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                    lifterUp(cone2);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{
                    gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                })
                // ---- motion----
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
                .build();

        TrajectorySequence Cone2PickUp = drive.trajectorySequenceBuilder(PreloadDrop.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,gripperClosed);
                }).waitSeconds(0.4)
                .addTemporalMarker(()->{
                    lifterUp(cone2+400);
                }).waitSeconds(0.1)
                //
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                    lifterUp(middle);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                })
                //---- motion -------
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
                .build();

        TrajectorySequence Cone2Drop = drive.trajectorySequenceBuilder(Cone1PickUp.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerIn);
                }).waitSeconds(.5)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                }).waitSeconds(DropDelay)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerZero);
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                    lifterUp(cone3);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{
                    gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                })
                // ---- motion----
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
                .build();

        TrajectorySequence Cone3PickUp = drive.trajectorySequenceBuilder(PreloadDrop.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,gripperClosed);
                }).waitSeconds(0.4)
                .addTemporalMarker(()->{
                    lifterUp(cone3+400);
                }).waitSeconds(0.1)
                //
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                    lifterUp(middle);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                })
                //---- motion -------
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1-0.3,-(24-3+1)-0.3,Math.toRadians(-135)),Math.toRadians(-135))
                .build();

        TrajectorySequence Cone3Drop = drive.trajectorySequenceBuilder(Cone1PickUp.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerIn);
                }).waitSeconds(.5)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                }).waitSeconds(DropDelay)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerZero);
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                    lifterUp(cone4);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{
                    gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                })
                // ---- motion----
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
                .build();

        TrajectorySequence Cone4PickUp = drive.trajectorySequenceBuilder(PreloadDrop.end())
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,gripperClosed);
                }).waitSeconds(0.4)
                .addTemporalMarker(()->{
                    lifterUp(cone4+400);
                }).waitSeconds(0.1)
                //
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                    lifterUp(middle);
                    gripping.setservo(gripping.servoBalancer,Constants.balancerOut);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.75,()->{
                    gripping.setservo(gripping.servorotate,Constants.rotatorDrop);
                })
                .UNSTABLE_addTemporalMarkerOffset(1,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderDrop);
                })
                //---- motion -------
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1 -0.3-0.3,-(24-3+1)-0.3-0.3,Math.toRadians(-135)),Math.toRadians(-135))
                .build();

        TrajectorySequence Cone4Drop = drive.trajectorySequenceBuilder(Cone1PickUp.end())
                .addTemporalMarker(()->{
                }).waitSeconds(.5)
                .addTemporalMarker(()->{
                    gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                }).waitSeconds(.2)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                .UNSTABLE_addTemporalMarkerOffset(0,()->{
                    gripping.setservo(gripping.servoBalancer,Constants.balancerZero);
                    gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                    gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                    lifterUp(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                    gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                })
                .UNSTABLE_addTemporalMarkerOffset(1.2,()->{
                    gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                })
                // ---- motion----
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24+12, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .build();

        TrajectorySequence TrackBack = drive.trajectorySequenceBuilder(startPose)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(24+11, -(24+12)), Math.toRadians(-90))
                .strafeTo(new Vector2d(24+11,-(24+24)))
                .splineToConstantHeading(new Vector2d(24+12+5,-(24+24+12+3)),Math.toRadians(0))
                .build();

        TrajectorySequence MovementTest = drive.trajectorySequenceBuilder(startPose)
// Moving to Pole
                .splineToConstantHeading(new Vector2d(24+11,-(24+24)),Math.toRadians(90))
                .strafeTo(new Vector2d(24+11,-(24+12)))
                .splineToConstantHeading(new Vector2d(24+5, -24), Math.toRadians(180))
////                              // Moving for picking up 1st Cone
                .setReversed(true)
//                .splineToConstantHeading(new Vector2d(24+12+5,-11.5),Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(24+12+2,-11.5),Math.toRadians(90))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
////                              // Moving back to Pole for Dropping 1st Cone
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
////                              // Moving for picking up 2nd cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
////                              // Moving back to Pole for Dropping 2nd Cone
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
                //3rd cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
////                              // Moving back to Pole for Dropping
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
                //4th cone
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(24+24, -11.5, Math.toRadians(180)), Math.toRadians(0))
                .lineTo(new Vector2d(24+24+12+3.5,-11.5))
////                              // Moving back to Pole for Dropping
                .setReversed(false)
                .lineTo(new Vector2d(24+24,-11.5))
                .splineToSplineHeading(new Pose2d(24+3+1,-(24-3+1),Math.toRadians(-135)),Math.toRadians(-135))
                .waitSeconds(5)
                .build();

        gripping.setservo(gripping.servorotate, Constants.rotatorPickUP);
        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
        gripping.setservo(gripping.servogripper, Constants.gripperClosed);
        gripping.setservo(gripping.servoBalancer, Constants.balancerZero);


        //      ########## Detecting Parking Zone ###########
        while (isStopRequested()==false && ParkingZone == "None" && isStarted() == false)
        {
            ArrayList<AprilTagDetection> currentDetections = AprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections)
                {
                    if (tag.id == eureka_IDS[PARKING_ZONE1] || tag.id == eureka_IDS[PARKING_ZONE2] || tag.id == eureka_IDS[PARKING_ZONE3])
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    getParkingZone(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                        getParkingZone(tagOfInterest);
                    }
                }
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                    getParkingZone(tagOfInterest);
                }
            }
            telemetry.update();
        }


        waitForStart();
        timer.reset();
        timer.startTime();

//######################## STARTS FROM HERE ################################
        drive.followTrajectorySequence(PreloadDrop);
//
        elapsedSeconds = timer.seconds();
        distanceAlign(drive,FrontAlign_target_Pole);
//
        elapsedSeconds = timer.seconds();
        poleAlign(drive);
//
        drive.update();
        drive.updatePoseEstimate();

        //parking
        if(ParkingZone == "1")
        {
            Cone4Drop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(()->{
                        lifterUp(middle - 400);
                        gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    }).waitSeconds(.2)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                    .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                        gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                        gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                        lifterUp(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,()->{
                        gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                    })
                    // ---- motion----
                    .setVelConstraint(fastConstraint)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(12, -11.5, Math.toRadians(0)), Math.toRadians(180))
                    .build();


        }
        else if(ParkingZone == "3")
        {
            Cone4Drop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(()->{
                        lifterUp(middle - 400);
                        gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    }).waitSeconds(.2)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                    .UNSTABLE_addTemporalMarkerOffset(0.2,()->{
                        gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                        gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                        lifterUp(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,()->{
                        gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                    })
                    // ---- motion----
                    .setVelConstraint(fastConstraint)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(24+24+12, -11.5, Math.toRadians(180)), Math.toRadians(0))
                    .build();
        }
        else
        {
            Cone4Drop = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .addTemporalMarker(()->{
                        lifterUp(middle - 200);
                        gripping.setservo(gripping.servogripper,Constants.gripperOpen);
                    }).waitSeconds(.2)
//                //PRELOAD DROP COMPLETED
//                //STARTING CONE SEQUENCE
                    .UNSTABLE_addTemporalMarkerOffset(0,()->{
                        gripping.setservo(gripping.servorotate,Constants.rotatorPickUP);
                        gripping.setservo(gripping.servogripper,Constants.gripperClosed);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.74,()->{
                        lifterUp(0);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(0.8,()->{
                        gripping.setservo(gripping.servoslide, Constants.sliderPickUp);
                    })
                    .UNSTABLE_addTemporalMarkerOffset(1,()->{
                        gripping.setservo(gripping.servogripper, Constants.gripperOpen);
                    })
                    // ---- motion----
                    .setVelConstraint(fastConstraint)
                    .setReversed(true)
                    .splineToSplineHeading(new Pose2d(24+12, -11.5, Math.toRadians(-90)), Math.toRadians(90))
                    .build();
        }

        drive.followTrajectorySequence(Cone4Drop);

        telemetry.update();
    }
    void lifterUp(double position)
    {
        liftleft.setTargetPosition((int) position);
        liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftleft.setPower(uppower);
        liftright.setTargetPosition((int) position);
        liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftright.setPower(uppower);
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



    void distanceAlignBack(SampleMecanumDrive drive) {
        while (opModeIsActive()) {
            if(timer.seconds() - elapsedSeconds >= bufferWaitTime) return;

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

    void colorAlignment(SampleMecanumDrive drive)
    {
        while (opModeIsActive()) {
            //timer
            if(timer.seconds() - elapsedSeconds >= bufferWaitTime) return;

            Pose2d poseEstimate = drive.getPoseEstimate();
            double heading = Math.toDegrees(poseEstimate.getHeading());
            double headingPower = gyroPID(heading, 180);

            if (Math.abs(headingPower) < 0.09) {
                headingPower = 0;
            }

            if (colorSensorLeft.blue() > threshold && colorSensorRight.blue() > threshold) {
                if(headingPower == 0) break;
                drive.setWeightedDrivePower(new Pose2d(0, 0, -headingPower));
                telemetry.addLine("at position");
            } else if (colorSensorLeft.blue() > threshold && colorSensorRight.blue() < threshold) {
                drive.setWeightedDrivePower(new Pose2d(0, -power,0));
            } else if (colorSensorLeft.blue() < threshold && colorSensorRight.blue() > threshold) {
                drive.setWeightedDrivePower(new Pose2d(0, power, 0));
            }

            drive.update();

            telemetry.addData("heading",heading);
            telemetry.addData("heading power",headingPower);
            telemetry.addData("color sensor left blue", colorSensorLeft.blue());
            telemetry.addData("color sensor right blue", colorSensorRight.blue());
            telemetry.addData("color sensor left", colorSensorLeft.red() + " " + colorSensorLeft.green() + " " + colorSensorLeft.blue());
            telemetry.addData("color sensor right", colorSensorLeft.red() + " " + colorSensorLeft.green() + " " + colorSensorLeft.blue());
            telemetry.update();
        }
    }

    void distanceAlign(SampleMecanumDrive drive,double FrontAlign_target)
    {
        while (opModeIsActive())
        {
            if(timer.seconds() - elapsedSeconds >= bufferWaitTime) return;
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

    void poleAlign(SampleMecanumDrive drive)
    {
        while (opModeIsActive())
        {
            if(timer.seconds() - elapsedSeconds >= bufferWaitTime) return;
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

    void tagToTelemetry(AprilTagDetection tagOfInterest){
        telemetry.addLine(String.format("\nDetected tag ID=%d", tagOfInterest.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", tagOfInterest.pose.x * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", tagOfInterest.pose.y * FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", tagOfInterest.pose.z * FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(tagOfInterest.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(tagOfInterest.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(tagOfInterest.pose.roll)));
    }

    void getParkingZone(AprilTagDetection tagOfInterest)
    {
        if(tagOfInterest.id == eureka_IDS[PARKING_ZONE1]){
            ParkingZone = "1";
        }
        else if(tagOfInterest.id == eureka_IDS[PARKING_ZONE2]){
            ParkingZone = "2";
        }
        else if(tagOfInterest.id == eureka_IDS[PARKING_ZONE3]){
            ParkingZone = "3";
        }
        telemetry.addData("parking zone is : ",ParkingZone);
    }

    void DetectParking()
    {
        ArrayList<AprilTagDetection> currentDetections = AprilTagDetectionPipeline.getLatestDetections();
        if (currentDetections.size() != 0)
        {
            boolean tagFound = false;
            for (AprilTagDetection tag : currentDetections)
            {
                if (tag.id == eureka_IDS[PARKING_ZONE1] || tag.id == eureka_IDS[PARKING_ZONE2] || tag.id == eureka_IDS[PARKING_ZONE3])
                {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if (tagFound)
            {
                telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                tagToTelemetry(tagOfInterest);
                getParkingZone(tagOfInterest);
            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");
            }
        }
//        sleep(10);
    }
}
