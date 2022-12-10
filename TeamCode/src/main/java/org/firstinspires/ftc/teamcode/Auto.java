package org.firstinspires.ftc.teamcode;

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

import java.util.List;

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
@Autonomous(name = "AutonomousTemp")
//@Disabled
public class Auto extends LinearOpMode {


    private static final String TFOD_MODEL_ASSET = "FullCone1209.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            "1 ball",
            "2 mapleLeaves",
            "3 popsicle"
    };



    private static final String VUFORIA_KEY =
            "AfRdm/v/////AAABmRa36nMVjkdAnYfT2LZQ510DdkArmXru8AlIRpqO3UtVZE3Z9ZYKytEimcLfDcA+0z0EXmct/ltDYeFFzpur3n0Vxc8+q0+T8vbFKi9a1evpD1yneH+7J958jn0+PIah5zmhySL7mbje2TyuQU9FAdfyLthRwy3oyBP781kdb8e9u9Vn+4Nltv/q9Wx8PN0a5IVWLV365Vx+F75ox6tgbEC/O7j/DD9BGpJte/DSdKALzBpHEm4pBex3nJnG78dboW6juB0BPjjNPdxn591qLWqSA06PrXI3L7ueMe4giIyqNvx8+IZetEhLnTTP0/JS7vSbsG3o1dpJiUgDf6MzTBugvJg/+6q/vd/J7oh30eg7";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        DcMotor leftFrontDrive  = hardwareMap.get(DcMotor.class, "motor4");
        DcMotor leftBackDrive  = hardwareMap.get(DcMotor.class, "motor3");
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "motor2");
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "motor1");
        DcMotorEx RightViperSlide = hardwareMap.get(DcMotorEx.class, "vpRight");
        DcMotorEx LeftViperSlide = hardwareMap.get(DcMotorEx.class, "vpLeft");
        Servo clawLeft = hardwareMap.servo.get("clwleft");
        Servo clawRight = hardwareMap.servo.get("clwright");
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        initVuforia();
        initTfod();

        RightViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftViperSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();


            tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        clawLeft.setPosition(0.0);
        clawRight.setPosition(0.2);
        sleep(500);
        RightViperSlide.setTargetPosition(1000);
        LeftViperSlide.setTargetPosition(-1000);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        LeftViperSlide.setVelocity(2000);
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
        telemetry.addData("finished", "true");
        telemetry.update();
        RightViperSlide.setTargetPosition(0);
        LeftViperSlide.setTargetPosition(0);
        RightViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LeftViperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RightViperSlide.setVelocity(2000);
        LeftViperSlide.setVelocity(2000);
//        leftBackDrive.setPower(0.5);
//        leftFrontDrive.setPower(0.5);
//        rightBackDrive.setPower(-0.5);
//        rightFrontDrive.setPower(-0.5);
//        sleep(500);
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftFrontDrive.setPower(0.1);
//        leftBackDrive.setPower(0.1);
//        rightBackDrive.setPower(0.1);
//        rightFrontDrive.setPower(0.1);
//        sleep(2000);
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftFrontDrive.setPower(-0.1);
//        leftBackDrive.setPower(-0.1);
//        rightBackDrive.setPower(-0.1);
//        rightFrontDrive.setPower(-0.1);
//        sleep(2000);
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
//        leftBackDrive.setPower(-0.5);
//        leftFrontDrive.setPower(-0.5);
//        rightBackDrive.setPower(0.5);
//        rightFrontDrive.setPower(0.5);
//        sleep(500);
//        leftFrontDrive.setPower(0);
//        leftBackDrive.setPower(0);
//        rightBackDrive.setPower(0);
//        rightFrontDrive.setPower(0);
        if (objectRecognized == "2 mapleLeaves") {
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(0.5);

            sleep(1200);

//            sleep(1300);

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            sleep(1100);
        } else if (objectRecognized == "1 ball") {
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            sleep(1850);
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(-0.5);
            leftBackDrive.setPower(-0.5);
            rightBackDrive.setPower(-0.5);
            rightFrontDrive.setPower(-0.5);
            sleep(1000);
        } else if (objectRecognized == "3 popsicle") {
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(-0.5);
            rightBackDrive.setPower(0.5);
            rightFrontDrive.setPower(-0.5);


            sleep(1200);

//            sleep(1000);


//            sleep(1200);

            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0.5);
            leftBackDrive.setPower(0.5);
            rightBackDrive.setPower(0.5);
            rightFrontDrive.setPower(0.5);
            sleep(1000);
        }
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);


    }

    /**
     * Initialize the Vuforia localization engine.
     */
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