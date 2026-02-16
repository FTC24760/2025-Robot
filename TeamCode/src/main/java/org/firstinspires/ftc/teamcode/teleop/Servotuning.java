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
    private Servo servo2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        servo = hardwareMap.get(Servo.class, "blockerL");
        servo2 = hardwareMap.get(Servo.class, "blockerR");



        double servoPosition1 = 0;
        double servoPosition2 = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // move servo to postion
            servo.setPosition(servoPosition1);
            servo2.setPosition(servoPosition2);

            // accept commands
            if (gamepad1.a) {
                if (servoPosition1 > 0) {
                    servoPosition1 -= 0.002;
                }
            } else if (gamepad1.x) {
                if (servoPosition1 < 1) {
                    servoPosition1 += 0.002;
                }
            } else if (gamepad1.b) {
                if (servoPosition2 > 0) {
                    servoPosition2 -= 0.002;
                }
            } else if (gamepad1.y) {
                if (servoPosition2 < 1) {
                    servoPosition2 += 0.002;
                }
            }



            telemetry.addData("Servo position1 real", servo.getPosition());
            telemetry.addData("Servo position1 button", servoPosition1);
            telemetry.addData("Servo position2 real", servo2.getPosition());
            telemetry.addData("Servo position2 button", servoPosition2);
            telemetry.update();

        }
    }
}
