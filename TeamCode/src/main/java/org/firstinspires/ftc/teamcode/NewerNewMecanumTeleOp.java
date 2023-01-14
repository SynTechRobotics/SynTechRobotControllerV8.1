/** This is the code used for the field-centric driving tutorial
 This is by no means a perfect code
 There are a number of improvements that can be made
 So, feel free to add onto this and make it better
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.util.AxisDirection;
import org.firstinspires.ftc.teamcode.util.BNO055IMUUtil;

import java.util.concurrent.TimeUnit;

/**
 * feel free to change the name or group of your class to better fit your robot
 */
@Disabled
@TeleOp(name = "DriverRelativeControl", group = "tutorial")
public class NewerNewMecanumTeleOp extends LinearOpMode {

    /**
     * make sure to change these motors to your team's preference and configuration
     */


    public BNO055IMU imu;

    Orientation angles;
    Acceleration gravity;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor backLeft = hardwareMap.dcMotor.get("motor3");
        DcMotor frontLeft = hardwareMap.dcMotor.get("motor4");
        DcMotor backRight = hardwareMap.dcMotor.get("motor1");
        DcMotor frontRight = hardwareMap.dcMotor.get("motor2");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");

        //        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "viperSlideRight");
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        // Servo servo3 = hardwareMap.servo.get("servo 3");


        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        RightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
//        Servo clawLeft = hardwareMap.servo.get("clawLeft");
//        Servo clawRight = hardwareMap.servo.get("clawRight");
        clawRight.setDirection(Servo.Direction.REVERSE);


        /**
         * you can change the variable names to make more sense
         */
        double driveTurn;
        //double driveVertical;
        //double driveHorizontal;

        double gamepadXCoordinate;
        double gamepadYCoordinate;
        double gamepadHypot;
        double gamepadDegree;
        double robotDegree;
        double movementDegree;
        double gamepadXControl;
        double gamepadYControl;

        /**
         * make sure to change this to how your robot is configured
         */

        //might need to change the motors being reversed
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        /**
         * make sure you've configured your imu properly and with the correct device name
         */
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        BNO055IMUUtil.remapZAxis(imu, AxisDirection.NEG_X);

        composeTelemetry();


        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);


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
            while (opModeIsActive()) {
                driveTurn = -gamepad1.right_stick_x;
                //driveVertical = -gamepad1.right_stick_y;
                //driveHorizontal = gamepad1.right_stick_x;

                gamepadXCoordinate = gamepad1.left_stick_x; //this simply gives our x value relative to the driver
                gamepadYCoordinate = -gamepad1.left_stick_y; //this simply gives our y vaue relative to the driver
                gamepadHypot = Range.clip(Math.hypot(gamepadXCoordinate, gamepadYCoordinate), 0, 1);
                //finds just how much power to give the robot based on how much x and y given by gamepad
                //range.clip helps us keep our power within positive 1
                // also helps set maximum possible value of 1/sqrt(2) for x and y controls if at a 45 degree angle (which yields greatest possible value for y+x)
                gamepadDegree = Math.atan2(gamepadYCoordinate, gamepadXCoordinate);
                //the inverse tangent of opposite/adjacent gives us our gamepad degree
                robotDegree = getAngle();
                //gives us the angle our robot is at
                movementDegree = gamepadDegree - robotDegree;
                //adjust the angle we need to move at by finding needed movement degree based on gamepad and robot angles
                gamepadXControl = Math.cos(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the adjacent side, we can get our needed x value to power our motors
                gamepadYControl = Math.sin(Math.toRadians(movementDegree)) * gamepadHypot;
                //by finding the opposite side, we can get our needed y value to power our motors

                /**
                 * again, make sure you've changed the motor names and variables to fit your team
                 */

                //by mulitplying the gamepadYControl and gamepadXControl by their respective absolute values, we can guarantee that our motor powers will not exceed 1 without any driveTurn
                //since we've maxed out our hypot at 1, the greatest possible value of x+y is (1/sqrt(2)) + (1/sqrt(2)) = sqrt(2)
                //since (1/sqrt(2))^2 = 1/2 = .5, we know that we will not exceed a power of 1 (with no turn), giving us more precision for our driving
                frontRight.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                backRight.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) + driveTurn);
                frontLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) + gamepadXControl * Math.abs(gamepadXControl) - driveTurn);
                backLeft.setPower(gamepadYControl * Math.abs(gamepadYControl) - gamepadXControl * Math.abs(gamepadXControl) - driveTurn);

            /*frontRight.setPower(driveVertical - driveHorizontal + driveTurn);
            backRight.setPower(driveVertical + driveHorizontal + driveTurn);
            frontLeft.setPower(driveVertical + driveHorizontal - driveTurn);
            backLeft.setPower(driveVertical - driveHorizontal - driveTurn);*/


// Viper Slide Code:


                if (gamepad1.x && useButtons) {
                    position = 1620;
                    goSlow = false;

                }
                if (gamepad1.y && useButtons) {
                    position = 2920;
                    goSlow = false;

                }
                if (gamepad1.b && useButtons) {
                    position = 4020;
                    goSlow = false;

                }
                if (gamepad1.a && useButtons) {
                    goSlow = false;
                    position = 0;
                }

                if (prevposition != position) {
                    RightViperSlide.setTargetPosition(-position);
                    LeftViperSlide.setTargetPosition(position);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    prevposition = position;
                    RightViperSlide.setVelocity(4000);
                    LeftViperSlide.setVelocity(4000);
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
                    position = 120;
                    goSlow = true;
                }
                if (gamepad1.dpad_left) {
                    position = 270;
                    goSlow = true;
                }
                if (gamepad1.dpad_up) {
                    position = 420;
                    goSlow = true;
                }
                if (gamepad1.dpad_right) {
                    position = 570;
                    goSlow = true;
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
    void composeTelemetry() {
        telemetry.addAction(new Runnable() {
            @Override
            public void run() {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                gravity = imu.getGravity();
            }
        });
    }

    //allows us to quickly get our z angle
    public double getAngle() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    }