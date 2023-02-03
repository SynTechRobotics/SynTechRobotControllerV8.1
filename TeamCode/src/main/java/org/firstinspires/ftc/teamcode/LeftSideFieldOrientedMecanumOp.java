package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.concurrent.TimeUnit;

@TeleOp
public class LeftSideFieldOrientedMecanumOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor1");
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        // Retrieve the IMU from the hardware map
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");

//        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        // Servo servo3 = hardwareMap.servo.get("servo 3");



        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");

        clawRight.setDirection(Servo.Direction.REVERSE);


        /*
         Reverse the right side motors
         Reverse left motors if you are using NeveRests
         motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
         motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        */

        waitForStart();

        if (isStopRequested()) return;



        boolean clawOpen = true;
        int position = 0;
        int prevPosition = 0;
        double x;
        double y;
        double rx;
        while (opModeIsActive()) {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            if (gamepad1.right_trigger > 0) {
                y = gamepad1.left_stick_y; // Remember, this is reversed!
                x = -gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = -gamepad1.right_stick_x;
            } else {
                y = 0.5*gamepad1.left_stick_y; // Remember, this is reversed!
                x = -0.5*gamepad1.left_stick_x; // Counteract imperfect strafing
                rx = -0.5*gamepad1.right_stick_x;
            }

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle-Math.toRadians(90);

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

//Viper Slide Code:


            if(gamepad1.x){
                position = 1200;

            }
            if(gamepad1.y) {
                position =  2100;

            }
            if(gamepad1.b) {
                position = 2950;

            }
            if(gamepad1.a) {
                position = 0;
            }

            if (prevPosition != position) {
                RightViperSlide.setTargetPosition(-position);
                LeftViperSlide.setTargetPosition(-position);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                prevPosition = position;
                RightViperSlide.setVelocity(4000);
                LeftViperSlide.setVelocity(4000);
            }

            if (gamepad1.dpad_down) {
                position = 100;
            }
            if (gamepad1.dpad_left) {
                position = 200;
            }
            if (gamepad1.dpad_up) {
                position = 300;
            }
            if (gamepad1.dpad_right) {
                position = 420;
            }

            //Servo Code
                if (gamepad1.right_bumper) {
                    if (clawOpen) {
                        clawLeft.setPosition(0.0);
                        clawRight.setPosition(0.7);
                        if (position < 150) {
                            position = 150;
                        }
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = false;
                    } else {
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(0.2);
                        if (position <= 150) {
                            position = 0;
                        }
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = true;
                    }
                }

                telemetry.addData("viperPos", position);
                telemetry.update();
            }


        }
    }