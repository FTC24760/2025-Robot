package org.firstinspires.ftc.teamcode.teleop.oldrobot;

import static java.lang.Math.floor;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Revolver/Indexer Servo Testing", group="Teleop")
public class IndexerServo extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo1, servo2, servo3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        servo1 = hardwareMap.get(Servo.class, "slot1");
        servo2 = hardwareMap.get(Servo.class, "slot2");
        servo3 = hardwareMap.get(Servo.class, "slot3");
        int indexerPos = 0;
        Servo servos[] = {servo1, servo2, servo3};
        double servoPos[] = {0, 0, 0};
        ElapsedTime[] lastActivated = {runtime, runtime, runtime};
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            /*for (int servo = 0; servo < 3; servo++) {
                servoPos[servo] = 0;
                if (lastActivated )
            }*/
            if((runtime.seconds()/5) - floor(runtime.seconds()/5) > 0.5)
                servo1.setPosition(0);
            else servo1.setPosition(0.15);

            telemetry.addData("Servo position", servo1.getPosition());
            telemetry.update();

        }
    }
}
