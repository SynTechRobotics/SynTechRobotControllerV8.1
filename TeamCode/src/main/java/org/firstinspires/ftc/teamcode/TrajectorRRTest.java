package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TrajectorRRTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        Pose2d startPose = new Pose2d(0, 0, 0);

        drive.setPoseEstimate(startPose);

        TrajectorySequence toHighJunction = drive.trajectorySequenceBuilder(startPose)
                .forward(4.5)
                .forward(48)
                .strafeLeft(12)
                .addDisplacementMarker(() -> {
                    // This marker runs after the first splineTo()
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.2);
                    // Run your action in here!
                })
                .strafeRight(12)
                .back(24)
                .build();
        waitForStart();
        clawLeft.setPosition(0.5);
        clawRight.setPosition(0.7);
        if (!isStopRequested()) drive.followTrajectorySequence(toHighJunction);
    }
}
