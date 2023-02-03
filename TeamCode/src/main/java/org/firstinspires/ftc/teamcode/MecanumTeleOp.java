package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;
@Disabled
@TeleOp
public class MecanumTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motor1");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motor2");
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LeftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

        boolean clawOpen = false;
        int position = 0;
        int prevPosition = 0;
        while (opModeIsActive()) {
//            servo2.setDirection(Servo.Direction.REVERSE);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            double y = -0.5*gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0.5*gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = 0.75*gamepad1.right_stick_x;
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


            if(gamepad1.x){
                position = 1700;
            }
            if(gamepad1.y) {
                position = 3000;
            }
            if(gamepad1.b) {
                position = 4100;
            }
            if(gamepad1.a) {
                position = 0;
            }

            if (prevPosition != position) {
//                LeftViperSlide.setTargetPosition(-position-200);
                RightViperSlide.setTargetPosition(position);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                prevPosition = position;
                RightViperSlide.setVelocity(2750);
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
            if (gamepad1.dpad_up || gamepad1.dpad_down) {
                if (gamepad1.dpad_up) {
                    position += 150;
                    if (position > 4100) {
                        position = 4100;
                    }
                    sleep(80);
                }
                if (gamepad1.dpad_down && position > 0) {
                    position -= 150;
                    if (position < 0) {
                        position = 0;
                    }
                    sleep(80);
                }
//                LeftViperSlide.setTargetPosition(-position-200);
                RightViperSlide.setTargetPosition(position);
//                TimeUnit.MILLISECONDS.sleep(10);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(1500);
//                LeftViperSlide.setVelocity(1500);
            }


           //Servo Code
            if (gamepad1.right_bumper) {
                if (clawOpen) {
                    clawLeft.setPosition(0.0);
                    clawRight.setPosition(0.7);
                    TimeUnit.MILLISECONDS.sleep(500);
                    clawOpen = false;
                } else {
                    clawLeft.setPosition(0.5);
                    clawRight.setPosition(0.2);
                    TimeUnit.MILLISECONDS.sleep(500);
                    clawOpen = true;
                }
            }

            }


        }
    }