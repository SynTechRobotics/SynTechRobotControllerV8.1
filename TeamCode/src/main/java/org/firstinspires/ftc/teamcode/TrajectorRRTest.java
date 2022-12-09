package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class TrajectorRRTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

         SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

         Pose2d startPose = new Pose2d(0, 0, 0);

    }
}
