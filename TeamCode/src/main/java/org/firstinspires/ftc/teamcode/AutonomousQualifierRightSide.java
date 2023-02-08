/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous
public class AutonomousQualifierRightSide extends LinearOpMode
{
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);
        int finalDetectionId = 0;

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        clawRight.setDirection(Servo.Direction.REVERSE);
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(-66, -38, 0);
        drive.setPoseEstimate(startPose);

        TrajectorySequence toMediumJunctionPos = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-53, -36, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(-9, -37))
                .lineToLinearHeading(new Pose2d(-24, -32, Math.toRadians(90)))
                .build();

        TrajectorySequence firstToConeStackPosition = drive.trajectorySequenceBuilder(toMediumJunctionPos.end())
                .lineToLinearHeading(new Pose2d(-13, -42, Math.toRadians(-90)))
                .lineToLinearHeading(new Pose2d(-14, -59, Math.toRadians(-90)))
                .build();

        TrajectorySequence firstToLowJunctionPos = drive.trajectorySequenceBuilder(firstToConeStackPosition.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(-14, -51, Math.toRadians(-200)))
                .forward(4)
                .build();

        TrajectorySequence secondToConeStackPosition1 = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .addDisplacementMarker(() -> {
                    RightViperSlide.setTargetPosition(-300);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightViperSlide.setVelocity(4000);
                })
                .lineToLinearHeading(new Pose2d(-15, -56, Math.toRadians(-90)))
                .forward(2.25)
                .build();

        TrajectorySequence firstToLowJunctionPos2 = drive.trajectorySequenceBuilder(secondToConeStackPosition1.end())
                .back(5)
                .lineToLinearHeading(new Pose2d(-14, -49, Math.toRadians(-200)))
                .forward(4)
                .build();

        TrajectorySequence secondToConeStackPosition2 = drive.trajectorySequenceBuilder(firstToLowJunctionPos2.end())
                .back(4)
                .addDisplacementMarker(() -> {
                    RightViperSlide.setTargetPosition(-200);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightViperSlide.setVelocity(4000);
                })
                .lineToLinearHeading(new Pose2d(-15, -56, Math.toRadians(-90)))
                .forward(2.75)
                .build();

        TrajectorySequence firstToLowJunctionPos3 = drive.trajectorySequenceBuilder(secondToConeStackPosition2.end())
                .back(7)
                .lineToLinearHeading(new Pose2d(-14, -48.5, Math.toRadians(-200)))
                .forward(4)
                .build();

        TrajectorySequence toRightPosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-14, -55.5, Math.toRadians(-90)))
                .build();

        TrajectorySequence toMiddlePosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-14, -33, Math.toRadians(-90)))
                .build();

        TrajectorySequence toLeftPosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .lineToLinearHeading(new Pose2d(-14, -9, Math.toRadians(-90)))
                .build();

        waitForStart();
        clawLeft.setPosition(0);
        clawRight.setPosition(0.7);
        sleep(700);
        RightViperSlide.setTargetPosition(-500);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2500);
        long start = System.currentTimeMillis();
        long end = start + 2000;
        while (!isStopRequested() && System.currentTimeMillis() < end) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 ) {
                finalDetectionId = currentDetections.get(0).id;
                break;
            }
        }
        //If the camera doesn't detect anything for 5 seconds, the finalDetectionId remains as 0.
        telemetry.addData("foundTag", "_" + finalDetectionId + "_");
        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        if (!isStopRequested()) {
            //to the low junction
            RightViperSlide.setTargetPosition(-2050);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(4000);
            drive.followTrajectorySequence(toMediumJunctionPos);
            clawLeft.setPosition(0.5);
            clawRight.setPosition(0.2);
            sleep(700);
            //to the Cone stack after
            RightViperSlide.setTargetPosition(-420);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(4000);
            drive.followTrajectorySequence(firstToConeStackPosition);
            clawLeft.setPosition(0);
            clawRight.setPosition(0.7);
            sleep(700);
            int x = 1;
            while (x <= 3 && !isStopRequested()) {
                RightViperSlide.setTargetPosition(-1200);
                RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                RightViperSlide.setVelocity(4000);
                sleep(500);
                if (x == 1) {
                    drive.followTrajectorySequence(firstToLowJunctionPos);
                } else if (x == 2) {
                    drive.followTrajectorySequence(firstToLowJunctionPos2);
                } else if (x == 3) {
                    drive.followTrajectorySequence(firstToLowJunctionPos3);
                }
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.2);
                sleep(600);
                if (x != 3) {
                    if (x == 1) {
                        drive.followTrajectorySequence(secondToConeStackPosition1);
                    } else if (x == 2) {
                        drive.followTrajectorySequence(secondToConeStackPosition2);
                    }
                    clawLeft.setPosition(0);
                    clawRight.setPosition(0.7);
                    sleep(700);
                } else {
                    RightViperSlide.setTargetPosition(0);
                    RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    RightViperSlide.setVelocity(4000);
                }
                x += 1;
            }
            if (finalDetectionId == 14) {
                drive.followTrajectorySequence(toLeftPosition);
            } else if (finalDetectionId == 15) {
                drive.followTrajectorySequence(toMiddlePosition);
            } else {
                drive.followTrajectorySequence(toRightPosition);
            }
        }
    }
}