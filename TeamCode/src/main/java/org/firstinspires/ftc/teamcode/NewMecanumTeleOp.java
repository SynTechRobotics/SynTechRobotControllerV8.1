package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class NewMecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
//        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
//        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
//        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");
//        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
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

        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
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
        boolean clawOpen = false;
        long prevInterval = starttime;
        int position = 0;
        int prevposition = 0;
        boolean clawfront = true;
        while (opModeIsActive()) {
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
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
                position = 1650;
            }
            if(gamepad1.y && useButtons) {
                position = 2950;
            }
            if(gamepad1.b && useButtons) {
                position = 4050;
            }
            if(gamepad1.a && useButtons) {
                goSlow = false;
                position = 0;
            }

            if (prevposition != position) {
                RightViperSlide.setTargetPosition(-position);
                LeftViperSlide.setTargetPosition(position);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                prevposition = position;
                if (goSlow) {
                    RightViperSlide.setVelocity(2000);
                    LeftViperSlide.setVelocity(2000);
                    goSlow = false;
                } else {
                    RightViperSlide.setVelocity(3500);
                    LeftViperSlide.setVelocity(3500);

                }
            }

            /* mode switch code (unused)
            if (gamepad1.dpad_left) {
                RightViperSlide.setTargetPosition(0);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2750);
                position = 0;
                useButtons = false;
                useIncrements = true;
            }
            if (gamepad1.dpad_right) {
                RightViperSlide.setTargetPosition(0);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2750);
                position = 0;
                useButtons = true;
                useIncrements = false;
            }
            */
//            if (gamepad1.dpad_up) {
//                position += 150;
//                if (position > 3000) {
//                    position = 3000;
//                }
//                sleep(80);
//            }
//            if (gamepad1.dpad_down && position > 0) {
//                position -= 150;
//                if (position < 0) {
//                    position = 0;
//                }
//                sleep(80);
//            }
//            goSlow = true;



            if (gamepad1.dpad_down) {
                position = 150;
                goSlow = true;
            }
            if (gamepad1.dpad_left) {
                position = 300;
                goSlow = true;
            }
            if (gamepad1.dpad_up) {
                position = 450;
                goSlow = true;
            }
            if (gamepad1.dpad_right) {
                position = 600;
                goSlow = true;
            }

            //Servo Code
                if (gamepad1.right_bumper) {
                    if (clawOpen) {
                        clawLeft.setPosition(0.0);
                        clawRight.setPosition(0.7);
                        position = 75;
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = false;
                    } else {
                        clawLeft.setPosition(0.5);
                        clawRight.setPosition(0.2);
                        position = 0;
                        TimeUnit.MILLISECONDS.sleep(500);
                        clawOpen = true;
                    }
                }

                telemetry.addData("viperPos", position);
                telemetry.update();
            }


        }
    }