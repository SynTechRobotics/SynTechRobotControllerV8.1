package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadRunnerRightSideCodeMeet3 extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        clawRight.setDirection(Servo.Direction.REVERSE);
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(new Pose2d(-60, -36, 0));

        TrajectorySequence fullTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-60, -36, 0))
                .forward(48)
                .turn(Math.toRadians(111))
                .forward(5)
                .back(5)
                .turn(Math.toRadians(-222))
//                .strafeLeft()
//                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
//                .forward(24)
//                .back(24)
//                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
//                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
//                .forward(24)
//                .back(24)
//                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
                .build();

        TrajectorySequence toConeStackPosition = drive.trajectorySequenceBuilder(startPose)
                .forward(48)
                .turn(Math.toRadians(90))
                .build();

        Trajectory toHighJunctionPosition = drive.trajectoryBuilder(startPose)
                .strafeRight(12)
                .build();

        Trajectory backToConeStack = drive.trajectoryBuilder(startPose)
                .strafeLeft(12)
                .build();



        TrajectorySequence backToStart = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(27)
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(28.5, 53))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(40, -47))
                .build();

        waitForStart();
        clawLeft.setPosition(0);
        clawRight.setPosition(0.7);
        RightViperSlide.setTargetPosition(0);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        sleep(1000);
        if (!isStopRequested()) {
            drive.followTrajectorySequence(fullTrajectory);
//            drive.followTrajectorySequence(toConeStackPosition);
//            RightViperSlide.setTargetPosition(4100);
//            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            RightViperSlide.setVelocity(3500);
//            sleep(2000);
//            drive.followTrajectory(toHighJunctionPosition);
//            clawLeft.setPosition(0.5);
//            clawRight.setPosition(0.7);
//            sleep(1000);
//            int x = 1;
//            while (x <= 3) {
//                drive.followTrajectory(backToConeStack);
//                RightViperSlide.setTargetPosition(0);
//                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                RightViperSlide.setVelocity(3500);
//                sleep(1000);
//                clawLeft.setPosition(0);
//                clawRight.setPosition(0.2);
//                sleep(1000);
//                RightViperSlide.setTargetPosition(4100);
//                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                RightViperSlide.setVelocity(3500);
//                sleep(1000);
//                drive.followTrajectory(toHighJunctionPosition);
//                clawLeft.setPosition(0.5);
//                clawRight.setPosition(0.7);
//                sleep(1000);
//                x++;
//            }
        }
    }
}
