package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Revolver Servo Position Tuning", group="Teleop")
public class ServoPositionTuning extends LinearOpMode {

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
    private Servo led1;
    private Servo led2;
    private Servo led3;
    private CRServo intake;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        servo = hardwareMap.get(Servo.class, "revolver");
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");
        intake = hardwareMap.get(CRServo.class, "intake");

        led1.setPosition(0.5);
        led2.setPosition(0.5);
        led3.setPosition(0.5);

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

            intake.setPower(1.0);

            telemetry.addData("Servo position real", servo.getPosition());
            telemetry.addData("Servo position button", servoPosition);
            telemetry.update();

        }
    }
}
