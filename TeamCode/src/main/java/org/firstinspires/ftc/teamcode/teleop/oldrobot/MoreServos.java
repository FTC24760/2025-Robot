package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="rotation tuning", group="Teleop")
@Disabled
public class MoreServos extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo leftRotate;
    private Servo rightRotate;
    private Servo led1;
    private Servo led2;
    private Servo led3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        leftRotate = hardwareMap.get(Servo.class, "leftRotate");
        rightRotate = hardwareMap.get(Servo.class, "rightRotate");
        led1 = hardwareMap.get(Servo.class, "led1");
        led2 = hardwareMap.get(Servo.class, "led2");
        led3 = hardwareMap.get(Servo.class, "led3");

        led1.setPosition(0.5);
        led2.setPosition(0.5);
        led3.setPosition(0.5);

        double leftServoPosition = 0;
        double rightServoPosition = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // move servo to postion
            leftRotate.setPosition(leftServoPosition);
            rightRotate.setPosition(rightServoPosition);

            // accept commands
            if (gamepad1.a) {
                if (leftServoPosition > 0) {
                    leftServoPosition -= 0.005;
                }
            } else if (gamepad1.y) {
                if (leftServoPosition < 1) {
                    leftServoPosition += 0.005;
                }
            }

            if (gamepad1.x) {
                if (rightServoPosition > 0) {
                    rightServoPosition -= 0.005;
                }
            } else if (gamepad1.b) {
                if (rightServoPosition < 1) {
                    rightServoPosition += 0.005;
                }
            }

            telemetry.addData("Servo position left", leftServoPosition);
            telemetry.addData("Servo position right", rightServoPosition);
            telemetry.update();

        }
    }
}
