package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

/**
 * This 2022-2023 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine which image is being presented to the robot.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "RobotVisionAutonomous2")
//@Disabled
public class AutonomousMeet2 extends LinearOpMode {

    DcMotorEx motorBackLeft;
    DcMotorEx motorBackRight;
    DcMotorEx motorFrontRight;
    DcMotorEx motorFrontLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        motorBackLeft =  hardwareMap.get(DcMotorEx.class, "motor3");
        motorFrontLeft = hardwareMap.get(DcMotorEx.class, "motor4");
        motorBackRight = hardwareMap.get(DcMotorEx.class, "motor1");
        motorFrontRight = hardwareMap.get(DcMotorEx.class, "motor2");

//        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
//        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        // Servo servo3 = hardwareMap.servo.get("servo 3");
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

//        LeftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

//        RightViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        LeftViperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
//        clawLeft.setPosition(0.1);
//        clawRight.setPosition(1);
        sleep(1500);
        Forward(1000, 1000);
        Left(500, 1000);

//      telemetry.addData("finished", "true");
        telemetry.update();
    }

    private void Forward(int time, int velocity) throws InterruptedException {
        motorBackLeft.setVelocity(-velocity);
        motorFrontRight.setVelocity(velocity);
        motorBackRight.setVelocity(velocity);
        motorFrontLeft.setVelocity(-velocity);
        TimeUnit.MILLISECONDS.sleep(time);
    }
    private void Left(int time, int velocity) throws InterruptedException {
        motorBackLeft.setVelocity(velocity*2);
        motorFrontRight.setVelocity(velocity*2);
        motorBackRight.setVelocity(-velocity*2);
        motorFrontLeft.setVelocity(-velocity*2);
        TimeUnit.MILLISECONDS.sleep(time);
    }
    private void Right(int time, int velocity) throws InterruptedException {
        motorBackLeft.setVelocity(-velocity*2);
        motorFrontRight.setVelocity(-velocity*2);
        motorBackRight.setVelocity(velocity*2);
        motorFrontLeft.setVelocity(velocity*2);
        TimeUnit.MILLISECONDS.sleep(time);
    }
    private void Back(int time, int velocity) throws InterruptedException {
        motorBackLeft.setVelocity(-velocity);
        motorFrontRight.setVelocity(-velocity);
        motorBackRight.setVelocity(-velocity);
        motorFrontLeft.setVelocity(-velocity);
        TimeUnit.MILLISECONDS.sleep(time);
    }
}

