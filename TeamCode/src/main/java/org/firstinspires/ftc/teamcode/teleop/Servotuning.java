package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="other Servo Position Tuning", group="Teleop")
public class Servotuning extends LinearOpMode {

    // Declare OpMode members.

    /*
    POSITIONS:
    slot 1 score - 0.075
slot 2 score - 0.805
slot 3 score - 0.45

slot 1 intake - 0.62
slot 2 intake - 0.245
slot 3 intake - 1

     */
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        servo = hardwareMap.get(Servo.class, "blocker");



        double servoPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // move servo to postion
            servo.setPosition(servoPosition);

            // accept commands
            if (gamepad1.a) {
                if (servoPosition > 0) {
                    servoPosition -= 0.002;
                }
            } else if (gamepad1.x) {
                if (servoPosition < 1) {
                    servoPosition += 0.002;
                }
            }


            telemetry.addData("Servo position real", servo.getPosition());
            telemetry.addData("Servo position button", servoPosition);
            telemetry.update();

        }
    }
}
