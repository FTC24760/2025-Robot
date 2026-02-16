package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="HOOD Servo Position Tuning", group="Teleop")
public class HoodServoTuning extends LinearOpMode {

    // Declare OpMode members.


    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo;
    private Servo servo2;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        servo = hardwareMap.get(Servo.class, "hood");



        double servoPosition1 = 0;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // move servo to postion
            servo.setPosition(servoPosition1);

            // accept commands
            if (gamepad1.a) {
                if (servoPosition1 > 0) {
                    servoPosition1 -= 0.002;
                }
            } else if (gamepad1.x) {
                if (servoPosition1 < 1) {
                    servoPosition1 += 0.002;
                }
            }



            telemetry.addData("Servo position1 real", servo.getPosition());
            telemetry.addData("Servo position1 button", servoPosition1);
            telemetry.update();

        }
    }
}
