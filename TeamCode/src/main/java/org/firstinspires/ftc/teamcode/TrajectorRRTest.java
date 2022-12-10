package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.List;

@Autonomous(name="AutonomousRoadRunner")
public class TrajectorRRTest extends LinearOpMode {
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

        drive.setPoseEstimate(startPose);
        Trajectory forward = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();

        Trajectory back = drive.trajectoryBuilder(startPose)
                .back(10)
                .build();

        TrajectorySequence toHighJunctionPosition = drive.trajectorySequenceBuilder(startPose)
                .forward(4.5)
                .forward(48)
                .back(24)
                .strafeLeft(25)
                .build();

        TrajectorySequence backtoStart = drive.trajectorySequenceBuilder(startPose)
                .strafeRight(25)
                .build();

        Trajectory left = drive.trajectoryBuilder(startPose)
                .strafeLeft(48)
                .build();

        Trajectory right = drive.trajectoryBuilder(startPose)
                .strafeRight(48)
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
        sleep(1000);
        RightViperSlide.setTargetPosition(1000);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        String objectRecognized = "";
        boolean firstObjectDetected = true;
        while (firstObjectDetected && !isStopRequested()) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
                    telemetry.addData("Object->", objectRecognized);
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
                        telemetry.addData("", " ");
                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        if (recognition.getLabel() != "1 ball" && recognition.getLabel() != "2 mapleLeaves" && recognition.getLabel() != "3 popsicle") {
                            objectRecognized = "";
                        } else if (firstObjectDetected && recognition.getConfidence() > 0.30) {
                            objectRecognized = recognition.getLabel();
                            firstObjectDetected = false;
                        }
                    }
                    telemetry.update();
                }
            }
        }
        RightViperSlide.setTargetPosition(0);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        sleep(1000);
        if (!isStopRequested()) {
            drive.followTrajectorySequence(toHighJunctionPosition);
            RightViperSlide.setTargetPosition(3000);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(2000);
            sleep(2000);
            drive.followTrajectory(forward);
            clawLeft.setPosition(0.5);
            clawRight.setPosition(0.7);
            sleep(2000);
            drive.followTrajectory(back);
            RightViperSlide.setTargetPosition(0);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(2000);
            sleep(2500);
            drive.followTrajectorySequence(backtoStart);
            if (objectRecognized == "2 mapleLeaves") {
                drive.followTrajectory(left);
            } else if (objectRecognized == "3 popsicle") {
                drive.followTrajectory(right);
            }
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
