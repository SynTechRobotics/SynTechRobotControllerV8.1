package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;



@TeleOp(group = "FINALCODE")
public class RobotRelativeMecanumTeleOp extends LinearOpMode {

    OpenCvWebcam webcam1 = null;
    String outputReal;
    class examplePipeline extends OpenCvPipeline {
        Mat YCbCr = new Mat();
        Mat Crop1;
        Mat Crop2;
        Mat Crop3;

        Mat outPut = new Mat();
        Scalar rect1Color = new Scalar(255.0, 255.0, 0.0);
        Scalar rect2Color = new Scalar(255.0, 255.0, 0.0);
        Scalar rect3Color = new Scalar(255.0, 255.0, 0.0);


        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);
            telemetry.addLine("pipeline running");

            Rect crop1 = new Rect(1, 1, 239, 1279);
            Rect crop2 = new Rect(240, 1, 239, 1279);
            Rect crop3 = new Rect(480, 1, 239, 1279);

            Imgproc.rectangle(outPut, crop1, rect1Color, 2);
            Imgproc.rectangle(outPut, crop2, rect2Color, 2);
            Imgproc.rectangle(outPut, crop3, rect3Color, 2);

            Crop1 = YCbCr.submat(crop1);
            Crop2 = YCbCr.submat(crop2);
            Crop3 = YCbCr.submat(crop3);

            Core.extractChannel(Crop1, Crop1, 1);
            Core.extractChannel(Crop2, Crop2, 1);
            Core.extractChannel(Crop3, Crop3, 1);

            Scalar leftavg = Core.mean(Crop1);
            Scalar midavg = Core.mean(Crop2);
            Scalar rightavg = Core.mean(Crop3);



            if (leftavg.val[0] > midavg.val[0] && leftavg.val[0] > rightavg.val[0]) {
                telemetry.addLine("Left");
                outputReal = "Left";
                rect1Color = new Scalar(0.0, 0.0, 255.0);
                rect2Color = new Scalar(255.0, 255.0, 0.0);
                rect3Color = new Scalar(255.0, 255.0, 0.0);
            } else if (midavg.val[0] > leftavg.val[0] && midavg.val[0] > rightavg.val[0]){
                telemetry.addLine("Middle");
                outputReal = "Middle";
                rect1Color = new Scalar(255.0, 255.0, 0.0);
                rect2Color = new Scalar(0.0, 0.0, 255.0);
                rect3Color = new Scalar(255.0, 255.0, 0.0);
            } else if (rightavg.val[0] > midavg.val[0] && rightavg.val[0] > leftavg.val[0]) {
                telemetry.addLine("Right");
                outputReal = "Right";
                rect1Color = new Scalar(255.0, 255.0, 0.0);
                rect2Color = new Scalar(255.0, 255.0, 0.0);
                rect3Color = new Scalar(0.0, 0.0, 255.0);
            }
            Imgproc.rectangle(outPut, crop1, rect1Color, 2);
            Imgproc.rectangle(outPut, crop2, rect2Color, 2);
            Imgproc.rectangle(outPut, crop3, rect3Color, 2);

            return(outPut);
        }
    }
    TrajectorySequence moveLeft;
    TrajectorySequence moveRight;
    TrajectorySequence moveBack;
    SampleMecanumDrive drive;
    Servo clawRight;
    Servo clawLeft;
    public void autoAllign() {
        if (outputReal != "Middle") {
            if (outputReal == "Left") {
                drive.followTrajectorySequence(moveRight);
            }
            if (outputReal == "Right") {
                drive.followTrajectorySequence(moveLeft);
            }
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {


        // Declare our motors
        // Make sure your ID's match your configuration
//        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
//        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam1 = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        webcam1.setPipeline(new examplePipeline());

        webcam1.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam1.startStreaming(1280, 720, OpenCvCameraRotation.SIDEWAYS_RIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });
        drive = new SampleMecanumDrive(hardwareMap);
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");

//        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        // Servo servo3 = hardwareMap.servo.get("servo 3");
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        clawLeft = hardwareMap.servo.get("clwleft");
        clawRight = hardwareMap.servo.get("clwright");
//        Servo clawLeft = hardwareMap.servo.get("clawLeft");
//        Servo clawRight = hardwareMap.servo.get("clawRight");
        clawRight.setDirection(Servo.Direction.REVERSE);


//        clawLeft.setPosition(0.1);
//        clawRight.setPosition(0.9);

        double lPosition = 0.1;
        double rPosition = 0.9;

        /*
         Reverse the right side motors
         Reverse left motors if you are using NeveRests
         motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
         motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        waitForStart();

        if (isStopRequested()) return;
        long starttime = System.currentTimeMillis();



        boolean useIncrements = false;
        boolean useButtons = true;
        int savedPosition = 0;
        boolean goFast = false;
        boolean goSlow = false;
        boolean clawOpen = true;
        long prevInterval = starttime;
        int position = 0;
        int prevposition = 0;
        boolean clawfront = true;
        double y;
        double x;
        double rx;

        while (opModeIsActive()) {
            moveLeft = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .strafeLeft(1)
                    .build();
            moveRight = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .strafeLeft(1)
                    .build();
            moveBack = drive.trajectorySequenceBuilder(new Pose2d(0, 0, 0))
                    .back(2)
                    .build();
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.right_trigger > 0) {
                y = -gamepad1.left_stick_y; // Remember, this is reversed!
                x = gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = gamepad1.right_stick_x;
            } else {
                y = -0.5*gamepad1.left_stick_y; // Remember, this is reversed!
                x = 0.5*gamepad1.left_stick_x*1.1; // Counteract imperfect strafing
                rx = 0.65*gamepad1.right_stick_x;
            }

            /*
             Denominator is the largest motor power (absolute value) or 1
             This ensures all the powers maintain the same ratio, but only when
             at least one is out of the range [-1, 1]
            */
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            motorFrontLeft.setPower(-frontLeftPower);
            motorBackLeft.setPower(-backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);


// Viper Slide Code:


            if(gamepad1.x && useButtons){
                position = 1200;
                goSlow = false;

            }
            if(gamepad1.y && useButtons) {
                position =  2023;
                goSlow = false;

            }
            if(gamepad1.b && useButtons) {
                position = 2850;
                goSlow = false;

            }
            if(gamepad1.a && useButtons) {
                goSlow = false;
                position = 0;
            }

            if (prevposition != position) {
                RightViperSlide.setTargetPosition(-position);
                LeftViperSlide.setTargetPosition(-position);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                prevposition = position;
                RightViperSlide.setVelocity(4000);
                LeftViperSlide.setVelocity(4000);
            }


            if (gamepad1.dpad_down) {
                position = 100;
                goSlow = true;
            }
            if (gamepad1.dpad_left) {
                position = 200;
                goSlow = true;
            }
            if (gamepad1.dpad_up) {
                position = 300;
                goSlow = true;
            }
            if (gamepad1.dpad_right) {
                position = 420;
                goSlow = true;
            }

            //Servo Code
                if (gamepad1.right_bumper) {
                    if (clawOpen) {
                        clawLeft.setPosition(0.0);
                        clawRight.setPosition(0.7);
                        if (position < 75) {
                            position = 75;
                        }
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = false;
                    } else {
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(0.2);
                        if (position <= 75) {
                            position = 0;
                        }
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = true;
                    }
                }
                }
        if (gamepad1.left_bumper) {
            if (clawOpen) {
                clawLeft.setPosition(1.1);
                clawRight.setPosition(0.8);
                if (position < 75) {
                    position = 75;
                }
                TimeUnit.MILLISECONDS.sleep(500);
                clawOpen = false;
            } else {
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.2);
                if (position <= 75) {
                    position = 0;
                }
                TimeUnit.MILLISECONDS.sleep(500);
                clawOpen = true;
            }
        }
        if (gamepad1.back) {
            autoAllign();
        }


        }
    }