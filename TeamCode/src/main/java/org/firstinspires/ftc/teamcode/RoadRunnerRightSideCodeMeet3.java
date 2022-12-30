package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class RoadRunnerRightSideCodeMeet3 extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "FullCone1209.tflite";

    private static final String[] LABELS = {
            "1 ball",
            "2 mapleLeaves",
            "3 popsicle"
    };

    private static final String VUFORIA_KEY =
            "AfRdm/v/////AAABmRa36nMVjkdAnYfT2LZQ510DdkArmXru8AlIRpqO3UtVZE3Z9ZYKytEimcLfDcA+0z0EXmct/ltDYeFFzpur3n0Vxc8+q0+T8vbFKi9a1evpD1yneH+7J958jn0+PIah5zmhySL7mbje2TyuQU9FAdfyLthRwy3oyBP781kdb8e9u9Vn+4Nltv/q9Wx8PN0a5IVWLV365Vx+F75ox6tgbEC/O7j/DD9BGpJte/DSdKALzBpHEm4pBex3nJnG78dboW6juB0BPjjNPdxn591qLWqSA06PrXI3L7ueMe4giIyqNvx8+IZetEhLnTTP0/JS7vSbsG3o1dpJiUgDf6MzTBugvJg/+6q/vd/J7oh30eg7";

    private VuforiaLocalizer vuforia;


    private TFObjectDetector tfod;

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(new Pose2d(-60, -36, 0));

        TrajectorySequence fullTrajectory = drive.trajectorySequenceBuilder(new Pose2d(-60, -36, 0))
                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
                .forward(24)
                .back(24)
                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
                .lineToLinearHeading(new Pose2d(-12, -36, Math.toRadians(270)))
                .forward(24)
                .back(24)
                .lineToLinearHeading(new Pose2d(-24, -36, Math.toRadians(90)))
                .build();

        TrajectorySequence toConeStackPosition = drive.trajectorySequenceBuilder(startPose)
                .forward(48)
                .turn(Math.toRadians(90))
                .build();

        Trajectory toHighJunctionPosition = drive.trajectoryBuilder(startPose)
                .strafeRight(12)
                .build();

        Trajectory backtoConeStack = drive.trajectoryBuilder(startPose)
                .strafeLeft(12)
                .build();



        TrajectorySequence backtoStart = drive.trajectorySequenceBuilder(startPose)
                .strafeLeft(27)
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(28.5, 53))
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .lineToConstantHeading(new Vector2d(40, -47))
                .build();


        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();


            tfod.setZoom(1.0, 16.0/9.0);
        }
        waitForStart();
        clawLeft.setPosition(0);
        clawRight.setPosition(0.2);
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
//                drive.followTrajectory(backtoConeStack);
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
    private void initVuforia() {

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");


        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.55f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);


        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}
