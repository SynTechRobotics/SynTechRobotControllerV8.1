package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.concurrent.TimeUnit;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
//    final static double RightOpen_MIN_RANGE = 0.8;
//    final static double RightOpen_MAX_RANGE = 0.5;
//    final static double RightClose_MIN_RANGE = 0.6;
//    final static double RightClose_MAX_RANGE = 0.8;
//    final static double LeftOpen_MIN_RANGE = 0.2;
//    final static double LeftOpen_MAX_RANGE = 0.5;
//    final static double LeftClose_MIN_RANGE = 0.2;
//    final static double LeftClose_MAX_RANGE = 0.4;
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
        // Servo servo3 = hardwareMap.servo.get("servo 3");
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

        double position = 0;
        boolean useIncrements = false;
        boolean useButtons = true;
        int savedPosition = 0;
        boolean bool1 = true;
        boolean clawOpen = false;
        while (opModeIsActive()) {
//            servo2.setDirection(Servo.Direction.REVERSE);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            // servo2.setDirection(Servo.Direction.REVERSE);
            double y = -0.5*gamepad1.left_stick_y; // Remember, this is reversed!
            double x = 0.5*gamepad1.left_stick_x * 1.0; // Counteract imperfect strafing
            double rx = 0.5*gamepad1.right_stick_x;
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
                position = 1;
                telemetry.addData("Pos", position);
                telemetry.update();
            }
            if(gamepad1.y && useButtons) {
                position = 2;
                telemetry.addData("Pos", position);
                telemetry.update();
            }
            if(gamepad1.b && useButtons) {
                position = 2.5;
                telemetry.addData("Pos", position);
                telemetry.update();
            }
            if(gamepad1.a && useButtons) {
                bool1 = true;
                position = 0;
                telemetry.addData("Pos", position);
                telemetry.update();
            }
            if (gamepad1.dpad_left) {
                RightViperSlide.setTargetPosition(0);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2750);
                savedPosition = 0;
                useButtons = false;
                useIncrements = true;
            }
            if (gamepad1.dpad_right) {
                RightViperSlide.setTargetPosition(0);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2750);
                savedPosition = 0;
                useButtons = true;
                useIncrements = false;
            }
            if (useIncrements) {
                if (gamepad1.dpad_up) {
                    savedPosition += 200;
                    TimeUnit.MILLISECONDS.sleep(140);
                }
                if (gamepad1.dpad_down && savedPosition > 0) {
                    savedPosition -= 200;
                    TimeUnit.MILLISECONDS.sleep(140);
                }
                RightViperSlide.setTargetPosition(savedPosition);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(1500);

            }
            if (position == 0 && useButtons) {
                if (bool1) {
                    RightViperSlide.setTargetPosition(0);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightViperSlide.setVelocity(2750);
                    bool1 = false;
                }
            }
            if (position == 1 && useButtons) {
                RightViperSlide.setTargetPosition(1700);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2500);
            }
            if (position == 2 && useButtons) {
                RightViperSlide.setTargetPosition(3000);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2500);
            }
            if (position == 3 && useButtons) {
                RightViperSlide.setTargetPosition(4100);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(2500);
            }
            telemetry.addData("pos", RightViperSlide.getCurrentPosition());
            telemetry.update();


            // Servo Code
//                if (gamepad1.left_bumper) {
//                    lPosition += 0.00001;
//                    rPosition -= 0.00001;
//                    rPosition = Range.clip(rPosition, RightOpen_MIN_RANGE, RightOpen_MAX_RANGE);
//                    lPosition = Range.clip(lPosition, LeftOpen_MIN_RANGE, LeftOpen_MAX_RANGE);
//                    clawRight.setPosition(rPosition);
//                    clawLeft.setPosition(lPosition);
//                telemetry.addData("lb L Claw", "%.2f", lPosition);
//                telemetry.addData("lb R Claw", "%.2f",rPosition);
//                telemetry.update();
//                }
//                if (gamepad1.right_bumper) {
//                    rPosition += 0.00001;
//                    lPosition -= 0.00001;
//                    rPosition = Range.clip(rPosition, RightClose_MIN_RANGE, RightClose_MAX_RANGE);
//                    lPosition = Range.clip(lPosition, LeftClose_MIN_RANGE, LeftClose_MAX_RANGE);
//                    clawLeft.setPosition(rPosition);
//                    clawRight.setPosition(lPosition);
//                telemetry.addData("rb L Claw", "%.2f", lPosition);
//                telemetry.addData("rb R Claw", "%.2f",rPosition);
//                telemetry.update();
//                }
//                telemetry.addData("savePos", "%.2f", lPosition);
//                telemetry.addData("savePos", "%.2f",rPosition);
//                telemetry.update();
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