package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class ValueFiddler extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        Servo clawFlip = hardwareMap.servo.get("clawFlipper");
        Servo clawTurn = hardwareMap.servo.get("clawTurner");
        int position = 0;
        boolean chosenServoIsFlip = false;
        waitForStart();
        while (opModeIsActive()) {
//            servo2.setDirection(Servo.Direction.REVERSE);
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least on
            if (gamepad1.right_bumper) {
                while (gamepad1.right_bumper) {
                    sleep(1);
                }
                position += 1;
            } else if (gamepad1.left_bumper) {
                while (gamepad1.left_bumper) {
                    sleep(1);
                }
                position -= 1;
            }
            if (gamepad1.a) {
                while (gamepad2.a) {
                    sleep(1);
                }
                if (chosenServoIsFlip) {
                    chosenServoIsFlip = false;
                } else {
                    chosenServoIsFlip = true;
                }
            }
            if (chosenServoIsFlip) {
                clawFlip.setPosition(position/10);
            } else {
                clawTurn.setPosition(position/10);
            }
            telemetry.addData("ClawTurnPosition", clawTurn.getPosition());
            telemetry.addData("ClawFlipPosition", clawFlip.getPosition());
            telemetry.update();
        }
    }
}