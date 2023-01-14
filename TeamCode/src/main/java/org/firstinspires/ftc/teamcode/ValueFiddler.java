package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Disabled
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
                position += 1;
                sleep(500);
            } else if (gamepad1.left_bumper) {
                position -= 1;
                sleep(500);
            }
            if (gamepad1.a) {
                if (chosenServoIsFlip) {
                    chosenServoIsFlip = false;
                } else {
                    chosenServoIsFlip = true;
                }
                sleep(500);
            }
            if (chosenServoIsFlip) {
                clawFlip.setPosition(position/10);
            } else {
                clawTurn.setPosition(position/10);
            }
            telemetry.addData("ClawTurnPosition", clawTurn.getPosition());
            telemetry.addData("ClawFlipPosition", clawFlip.getPosition());
            telemetry.addData("ClawFlipClosen", chosenServoIsFlip);
            telemetry.addData("position", position);
            telemetry.update();
        }
    }
}