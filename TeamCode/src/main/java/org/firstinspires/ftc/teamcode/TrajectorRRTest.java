package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="TrajectoryRRTest")
public class TrajectorRRTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();
        Trajectory back = drive.tr
        TrajectorySequence toHighJunctionPosition = drive.trajectorySequenceBuilder(startPose)
                .forward(4.5)
                .forward(24)
                .strafeLeft(25)
//                .addDisplacementMarker(() -> {
//
//                    RightViperSlide.setTargetPosition(3000);
//                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    RightViperSlide.setVelocity(2000);
//
//                })
                .forward(10)
//                .addDisplacementMarker(93.5, () -> {
//
//                    while (RightViperSlide.getCurrentPosition() < 3000) {
//                    }
//                    clawLeft.setPosition(0.5);
//                    clawRight.setPosition(0.7);
//                    while (clawLeft.getPosition() < 0.5) {
//                    }
//
//                })
                .back(10)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    RightViperSlide.setTargetPosition(0);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightViperSlide.setVelocity(2000);
                    sleep(3000);
                    // Run your action in here!
                })
                .strafeRight(25)
                .back(24)
                .build();
        waitForStart();
        clawLeft.setPosition(0);
        clawRight.setPosition(0.2);
        sleep(1000);
        if (!isStopRequested()) drive.followTrajectorySequence(toHighJunction);
    }
}
