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
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.List;

@Autonomous(name="AutonomousRoadRunner")
public class RoadRunnerLeftSideCode extends LinearOpMode {
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
        Pose2d afterPositioning = new Pose2d(28.5, -27, 0);
        Pose2d afterForward = new Pose2d(36.5, 25, 0);
        Pose2d afterBacktoStart = new Pose2d(38.5, 0, 0);
        drive.setPoseEstimate(startPose);
        Trajectory forward = drive.trajectoryBuilder(afterPositioning)
                .forward(6.5)
                .build();

        Trajectory back = drive.trajectoryBuilder(afterForward)
                .back(6.5)
                .build();

        TrajectorySequence toHighJunctionPosition = drive.trajectorySequenceBuilder(startPose)
                .forward(52.5)
                .back(24)
                .strafeRight(27)
                .build();

        TrajectorySequence backtoStart = drive.trajectorySequenceBuilder(afterPositioning)
                .strafeLeft(27)
                .build();

        Trajectory left = drive.trajectoryBuilder(afterPositioning)
                .lineToConstantHeading(new Vector2d(28.5, 53))
                .build();

        Trajectory right = drive.trajectoryBuilder(afterBacktoStart)
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
        sleep(1000);
        RightViperSlide.setTargetPosition(1000);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        List<String> objectRecognizedList = new ArrayList<>();
        long start = System.currentTimeMillis();
        long end = start + 3000;
        while (objectRecognizedList.size() < 4 && !isStopRequested() && System.currentTimeMillis() < end) {
            if (tfod != null) {
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
//                    telemetry.addData("# Objects Detected", updatedRecognitions.size());
//                    telemetry.addData("Object->", objectRecognizedList);
                    for (Recognition recognition : updatedRecognitions) {
                        double col = (recognition.getLeft() + recognition.getRight()) / 2;
                        double row = (recognition.getTop() + recognition.getBottom()) / 2;
                        double width = Math.abs(recognition.getRight() - recognition.getLeft());
                        double height = Math.abs(recognition.getTop() - recognition.getBottom());
//                        telemetry.addData("", " ");
//                        telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
//                        telemetry.addData("- Position (Row/Col)", "%.0f / %.0f", row, col);
//                        telemetry.addData("- Size (Width/Height)", "%.0f / %.0f", width, height);
                        if (recognition.getConfidence() > 0.60 && !objectRecognizedList.contains(recognition.getLabel()+'Z'+recognition.getConfidence())) {
                            objectRecognizedList.add(recognition.getLabel()+'Z'+recognition.getConfidence());
                        }
                    }
//                    telemetry.update();
                }
            }
        }
        RightViperSlide.setTargetPosition(0);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        int xVal = 0;
        int mapleLeavesCount = 0;
        int ballCount = 0;
        int popsicleCount = 0;
        float mapleLeafConfidence = 0;
        float ballConfidence = 0;
        float popsicleConfidence = 0;
        String trueObjectRecognized = "2 mapleLeaves";
        while (xVal < objectRecognizedList.size()) {
            String[] label = objectRecognizedList.get(xVal).split("Z");
            if(label[0].equals("2 mapleLeaves")) {
                mapleLeavesCount += 1;
                if(mapleLeafConfidence > Float.parseFloat(label[1])) {
                    mapleLeafConfidence = Float.parseFloat(label[1]);
                }
            } else if(label[0].equals("1 ball")) {
                ballCount += 1;
                if(ballConfidence > Float.parseFloat(label[1])) {
                    ballConfidence = Float.parseFloat(label[1]);
                }
            } else if(objectRecognizedList.get(xVal).startsWith("3 popsicle")) {
                popsicleCount += 1;
                if(popsicleConfidence > Float.parseFloat(label[1])) {
                    popsicleConfidence = Float.parseFloat(label[1]);
                }
            }
            xVal += 1;
        }

        if (mapleLeavesCount > ballCount && mapleLeavesCount > popsicleCount) {
            trueObjectRecognized = "2 mapleLeaves";
        } else if (ballCount > mapleLeavesCount && ballCount > popsicleCount) {
            trueObjectRecognized = "1 ball";
        } else if (popsicleCount > mapleLeavesCount && popsicleCount > ballCount) {
            trueObjectRecognized = "3 popsicle";
        } else if (mapleLeafConfidence > ballConfidence && mapleLeafConfidence > popsicleConfidence) {
            trueObjectRecognized = "2 mapleLeaves";
        } else if (ballConfidence > mapleLeafConfidence && ballConfidence > popsicleConfidence) {
            trueObjectRecognized = "1 ball";
        } else if (popsicleConfidence > mapleLeafConfidence && popsicleConfidence > ballConfidence) {
            trueObjectRecognized = "3 popsicle";
        } else {
            trueObjectRecognized = "2 mapleLeaves";
        }
        telemetry.addData("trueObjectRecognized", trueObjectRecognized);
        telemetry.update();
        RightViperSlide.setTargetPosition(0);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        sleep(1000);
        if (!isStopRequested()) {
            drive.followTrajectorySequence(toHighJunctionPosition);
            RightViperSlide.setTargetPosition(3000);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(3000);
            sleep(1000);
            drive.followTrajectory(forward);
            clawLeft.setPosition(0.5);
            clawRight.setPosition(0.7);
            sleep(1000);
            drive.followTrajectory(back);
            RightViperSlide.setTargetPosition(0);
            RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            RightViperSlide.setVelocity(3000);
            sleep(1000);
            if (trueObjectRecognized == "2 mapleLeaves") {
                drive.followTrajectory(left);
            } else {
                drive.followTrajectorySequence(backtoStart);
                if (trueObjectRecognized == "3 popsicle") {
                    drive.followTrajectory(right);
                }
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
