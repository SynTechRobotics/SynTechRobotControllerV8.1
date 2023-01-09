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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class AutonomousMeet3RightSide extends LinearOpMode
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
    int LEFT = 14;
    int MIDDLE = 15;
    int RIGHT = 16;
    AprilTagDetection tagOfInterest = null;

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
        String finalDetectionHashCode = new String();
        String finalDetectionPose = new String();
        int finalDetectionId = 0;
        /** Wait for the game to begin */

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        clawRight.setDirection(Servo.Direction.REVERSE);
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        LeftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(-60, -36, 0);
        drive.setPoseEstimate(startPose);
        int correctHeight = 600;

        TrajectorySequence firstToLowJunctionPos = drive.trajectorySequenceBuilder(startPose)
                .forward(4)
                .turn(Math.toRadians(-90))
                .strafeLeft(40)
                .forward(4)
                .build();

        TrajectorySequence firstToConeStackPosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .strafeLeft(25)
                .strafeRight(12)
                .forward(24.75)
                .build();

        TrajectorySequence secondToLowJunctionPos = drive.trajectorySequenceBuilder(firstToConeStackPosition.end())
                .back(24.75)
                .strafeRight(14)
                .forward(4)
                .build();

        TrajectorySequence secondToConeStackPosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .strafeLeft(13.5)
                .forward(24.5)
                .build();

        TrajectorySequence toRightPosition = drive.trajectorySequenceBuilder(secondToLowJunctionPos.end())
                .back(4)
                .strafeLeft(13.5)
                .forward(22)
                .build();

        TrajectorySequence toMiddlePosition = drive.trajectorySequenceBuilder(firstToLowJunctionPos.end())
                .back(4)
                .strafeLeft(13.5)
                .build();

        TrajectorySequence toLeftPosition = drive.trajectorySequenceBuilder(secondToLowJunctionPos.end())
                .back(4)
                .strafeLeft(13.5)
                .back(24.75)
                .build();

        waitForStart();
        clawLeft.setPosition(0);
        clawRight.setPosition(0.7);
        sleep(1000);
        LeftViperSlide.setTargetPosition(500);
        LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftViperSlide.setVelocity(2500);
        while (!isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0 ) {
                finalDetectionId = currentDetections.get(0).id;
                break;
            }
        }
        telemetry.addData("foundTag", "_" + String.valueOf(finalDetectionId) + "_");
        telemetry.update();
        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        if (!isStopRequested()) {
            LeftViperSlide.setTargetPosition(1700);
            LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftViperSlide.setVelocity(3000);
            drive.followTrajectorySequence(firstToLowJunctionPos);
            clawLeft.setPosition(0.5);
            clawRight.setPosition(0.2);
            sleep(1000);
            LeftViperSlide.setTargetPosition(600);
            LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LeftViperSlide.setVelocity(3000);
            drive.followTrajectorySequence(firstToConeStackPosition);
            clawLeft.setPosition(0);
            clawRight.setPosition(0.7);
            sleep(1000);
            int x = 1;
                // Back a bit
                LeftViperSlide.setTargetPosition(1700);
                LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                LeftViperSlide.setVelocity(2000);
                sleep(500);
                drive.followTrajectorySequence(secondToLowJunctionPos);
                // Dropping the cone and grabbing another one
                clawLeft.setPosition(0.5);
                clawRight.setPosition(0.2);
                sleep(1000);
                if (finalDetectionId == 14) {
                    LeftViperSlide.setTargetPosition(450);
                    LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftViperSlide.setVelocity(3000);
                    drive.followTrajectorySequence(toLeftPosition);
                } else if (finalDetectionId == 15) {
                    LeftViperSlide.setTargetPosition(450);
                    LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftViperSlide.setVelocity(3000);
                    drive.followTrajectorySequence(toMiddlePosition);
                } else if (finalDetectionId == 16){
                    LeftViperSlide.setTargetPosition(450);
                    LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    LeftViperSlide.setVelocity(3000);
                    drive.followTrajectorySequence(secondToConeStackPosition);
                }

        }
    }
}